#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use core::future::Future;
use core::pin::Pin;

use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use infrared::protocol::Nec;
use infrared::receiver::BufferInputReceiver;
use infrared::sender::ProtocolEncoder;
use xpanse_api::{
    bus::allocator::BusAllocator,
    driver::{Driver, DriverError, DriverMeta},
    gpio_bank::{BankPins, GpioBank},
    metadata::{ModuleDetectResistor, ModuleID, ModuleSlot},
    reexports::embassy_time::{Duration, Instant, Timer, with_timeout},
    registry::Registry,
};

pub use infrared::protocol::nec::NecCommand;

/// Carrier frequency the TSOP38238 demodulates.
const CARRIER_HZ: u32 = 38_000;
/// The carrier is on for a third of each period, the usual duty for IR remotes.
const CARRIER_DUTY_NUMERATOR: u32 = 1;
const CARRIER_DUTY_DENOMINATOR: u32 = 3;

/// All pulse durations, sent and captured, are in microseconds.
const TIMEBASE_HZ: u32 = 1_000_000;

/// The IR LEDs are rated for 100 mA only in short pulses (datasheet: 1/10 duty,
/// 0.1 ms pulse width). No legitimate frame holds the carrier on this long, so
/// a longer mark means a bug or corrupt data and is clamped rather than sent.
const MAX_MARK_US: u32 = 10_000;

/// A frame is finished once the receiver has been idle this long.
const FRAME_GAP_MS: u64 = 20;

/// Longest raw capture the module will record.
pub const CAPTURE_LEN: usize = 128;

/// Error returned by the module's IR capabilities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrError {
    /// No IR activity arrived before the timeout expired.
    Timeout,
    /// A burst was captured but did not decode as a valid command.
    Undecodable,
}

/// Async interface for transmitting IR.
///
/// Apps lease it from the registry as `Box<dyn IrTransmitter>`.
pub trait IrTransmitter: Send {
    /// Transmit one NEC command.
    fn send_nec<'a>(&'a mut self, address: u8, command: u8)
    -> Pin<Box<dyn Future<Output = ()> + 'a>>;

    /// Transmit raw mark/space durations in microseconds, starting with a mark.
    ///
    /// Use this to replay a capture from [`IrReceiver::capture`] without caring
    /// which protocol it is.
    fn send_raw<'a>(&'a mut self, durations: &'a [u32])
    -> Pin<Box<dyn Future<Output = ()> + 'a>>;
}

/// Async interface for receiving IR.
///
/// Apps lease it from the registry as `Box<dyn IrReceiver>`.
pub trait IrReceiver: Send {
    /// Record one burst of raw mark/space durations in microseconds.
    ///
    /// Returns how many entries of `buf` were filled. The first entry is a
    /// mark. Waits up to `timeout_ms` for the burst to start; the burst ends
    /// after [`FRAME_GAP_MS`] of silence.
    fn capture<'a>(
        &'a mut self,
        buf: &'a mut [u32],
        timeout_ms: u64,
    ) -> Pin<Box<dyn Future<Output = Result<usize, IrError>> + 'a>>;

    /// Wait for one NEC command and decode it.
    fn receive_nec<'a>(
        &'a mut self,
        timeout_ms: u64,
    ) -> Pin<Box<dyn Future<Output = Result<NecCommand, IrError>> + 'a>>;
}

/// Transmitter built on one PWM channel generating the 38 kHz carrier.
///
/// Marks and spaces are timed by the async executor: the carrier is gated by
/// setting the PWM compare value, which is a register write, so the pin is
/// never left high between frames.
struct PwmIrTransmitter {
    pwm: Pwm<'static>,
    config: PwmConfig,
    /// Compare value producing the carrier duty cycle.
    mark_compare: u16,
}

impl PwmIrTransmitter {
    fn new(pwm: Pwm<'static>, config: PwmConfig, mark_compare: u16) -> Self {
        Self {
            pwm,
            config,
            mark_compare,
        }
    }

    /// Start or stop the carrier. Compare 0 holds the output low.
    fn set_carrier(&mut self, on: bool) {
        self.config.compare_a = if on { self.mark_compare } else { 0 };
        self.pwm.set_config(&self.config);
    }

    /// Play out alternating mark/space durations, starting with a mark.
    ///
    /// Deadlines are absolute so a late wakeup does not push every later edge
    /// out with it.
    async fn play(&mut self, durations: &[u32]) {
        let mut deadline = Instant::now();

        for (index, &duration) in durations.iter().enumerate() {
            let is_mark = index % 2 == 0;
            let duration = if is_mark {
                duration.min(MAX_MARK_US)
            } else {
                duration
            };

            self.set_carrier(is_mark);
            deadline += Duration::from_micros(duration as u64);
            Timer::at(deadline).await;
        }

        // Never leave the LEDs lit, whatever the buffer contained
        self.set_carrier(false);
    }
}

impl IrTransmitter for PwmIrTransmitter {
    fn send_nec<'a>(
        &'a mut self,
        address: u8,
        command: u8,
    ) -> Pin<Box<dyn Future<Output = ()> + 'a>> {
        Box::pin(async move {
            let cmd = NecCommand {
                addr: address,
                cmd: command,
                repeat: false,
            };

            let mut buf = [0u32; CAPTURE_LEN];
            let len = <Nec as ProtocolEncoder<TIMEBASE_HZ>>::encode(&cmd, &mut buf);
            self.play(&buf[..len]).await;
        })
    }

    fn send_raw<'a>(
        &'a mut self,
        durations: &'a [u32],
    ) -> Pin<Box<dyn Future<Output = ()> + 'a>> {
        Box::pin(async move { self.play(durations).await })
    }
}

/// Receiver built on the TSOP38238's demodulated output.
///
/// The output is active-low and open-drain; the internal pull-up idles it high.
struct TsopIrReceiver {
    pin: Input<'static>,
}

impl TsopIrReceiver {
    async fn capture_into(&mut self, buf: &mut [u32], timeout_ms: u64) -> Result<usize, IrError> {
        if buf.is_empty() {
            return Ok(0);
        }

        // A burst starts when the demodulator pulls its output low
        with_timeout(
            Duration::from_millis(timeout_ms),
            self.pin.wait_for_falling_edge(),
        )
        .await
        .map_err(|_| IrError::Timeout)?;

        let mut last_edge = Instant::now();
        let mut count = 0;

        while count < buf.len() {
            // Silence for a whole gap means the burst is over
            let edge = with_timeout(
                Duration::from_millis(FRAME_GAP_MS),
                self.pin.wait_for_any_edge(),
            )
            .await;

            let now = Instant::now();
            match edge {
                Ok(()) => {
                    buf[count] = (now - last_edge).as_micros() as u32;
                    count += 1;
                    last_edge = now;
                }
                Err(_) => break,
            }
        }

        Ok(count)
    }
}

impl IrReceiver for TsopIrReceiver {
    fn capture<'a>(
        &'a mut self,
        buf: &'a mut [u32],
        timeout_ms: u64,
    ) -> Pin<Box<dyn Future<Output = Result<usize, IrError>> + 'a>> {
        Box::pin(async move { self.capture_into(buf, timeout_ms).await })
    }

    fn receive_nec<'a>(
        &'a mut self,
        timeout_ms: u64,
    ) -> Pin<Box<dyn Future<Output = Result<NecCommand, IrError>> + 'a>> {
        Box::pin(async move {
            let mut buf = [0u32; CAPTURE_LEN];
            let len = self.capture_into(&mut buf, timeout_ms).await?;

            let mut decoder: BufferInputReceiver<Nec> =
                BufferInputReceiver::with_frequenzy(TIMEBASE_HZ);

            decoder.iter(&buf[..len]).next().ok_or(IrError::Undecodable)
        })
    }
}

pub struct IrDriver;

impl DriverMeta for IrDriver {
    // MD0 through R1 (1k), MD1 through R2 (33k)
    const ID: ModuleID = ModuleID {
        md0: ModuleDetectResistor::R1K,
        md1: ModuleDetectResistor::R1K1,
    };
}

impl<G: BankPins> Driver<G> for IrDriver {
    async fn create(
        gpio_bank: GpioBank<G>,
        slot: ModuleSlot,
        registry: &mut Registry,
        bus_allocator: &mut BusAllocator,
    ) -> Result<(), DriverError> {
        // The IR LEDs hang off GPIO5, which is channel A of this bank's second
        // PWM slice, so the carrier comes from hardware rather than a PIO block
        let top = (clk_sys_freq() / CARRIER_HZ - 1) as u16;
        let mark_compare =
            ((top as u32 + 1) * CARRIER_DUTY_NUMERATOR / CARRIER_DUTY_DENOMINATOR) as u16;

        let mut config = PwmConfig::default();
        config.top = top;
        // Start with the carrier off
        config.compare_a = 0;

        let pwm = Pwm::new_output_a(gpio_bank.pwm_slice1, gpio_bank.gpio5, config.clone());

        registry.register(
            slot,
            Self::ID,
            Box::new(PwmIrTransmitter::new(pwm, config, mark_compare)) as Box<dyn IrTransmitter>,
        );

        // TSOP38238 output on GPIO6, idling high through the internal pull-up
        registry.register(
            slot,
            Self::ID,
            Box::new(TsopIrReceiver {
                pin: Input::new(gpio_bank.gpio6, Pull::Up),
            }) as Box<dyn IrReceiver>,
        );

        // Carrier comes from PWM, capture from a plain GPIO input: no buses
        let _ = bus_allocator;

        Ok(())
    }
}
