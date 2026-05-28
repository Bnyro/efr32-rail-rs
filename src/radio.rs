use core::ffi::c_void;

use efr32mg22_pac::{Peripherals, interrupt};

use crate::error::{IntoRailResult, RailError, RailResult};
use crate::rail::*;

pub const MAX_PACKET_LENGTH_BYTES: usize = 4096;

// FIFO BUFFER has to survive the whole program execution, so we have to declare it
// as static. Otherwise, if it'd be removed from the stack, RAIL could no longer
// write received packets into it.
static mut FIFO_BUFFER: [u32; MAX_PACKET_LENGTH_BYTES / 4] = [0; MAX_PACKET_LENGTH_BYTES / 4];

unsafe extern "C" fn events_callback(rail_handle: sl_rail_handle_t, event: sl_rail_events_t) {
    // ... handle RAIL events, e.g., receive and transmit completion
    if event == SL_RAIL_EVENT_RX_PACKET_RECEIVED.into() {
        #[cfg(feature = "defmt-logging")]
        defmt::info!("received packet");

        let radio = Radio::from(rail_handle);

        // Keep the received packet inside the queue and don't automatically flush it.
        radio.hold_packet();

        // trigger receive listener
        radio.trigger_receive_callback();
    }
}

unsafe extern "C" fn init_callback(_c: RAIL_Handle_t) {
    #[cfg(feature = "defmt-logging")]
    defmt::info!("successfully initialized radio");
}

#[derive(Clone, Copy)]
pub struct RadioConfig {
    pub tx_power_dbm: i16,
    pub packet_length_bytes: u16,
    pub on_packet_received: fn(),
}

pub struct Radio {
    rail_handle: sl_rail_handle_t,
    radio_config: RadioConfig,
}

impl From<sl_rail_handle_t> for Radio {
    fn from(rail_handle: sl_rail_handle_t) -> Self {
        let rail_config = unsafe { sl_rail_get_config(rail_handle) };
        let radio_config = unsafe { (*rail_config).p_opaque_handle1 as *mut RadioConfig };

        Radio {
            rail_handle,
            // TODO: this copy is probably inefficient here
            radio_config: unsafe { *radio_config },
        }
    }
}

impl Radio {
    /// Configure required clocks for starting the Radio.
    pub fn configure_clocks(peripherals: &Peripherals) {
        // enable HFXO Clock
        peripherals.cmu_ns.clken0().write(|w| w.hfxo0().set_bit());
        peripherals.hfxo0_ns.ctrl().write(|w| w.forceen().set_bit());
        // wait until the clock finished starting
        while peripherals.hfxo0_ns.status().read().rdy().bit_is_clear() {}
        // set sysclk to HFXO Clock - this is required according to https://docs.silabs.com/rail/latest/rail-api/efr32-main#high-frequency-clocks
        // otherwise sl_rail_configure_channels will crash
        peripherals.cmu_ns.sysclkctrl().write(|w| w.clksel().hfxo());
    }

    /// Create and initialize the radio. This immediately starts listening for packets.
    ///
    /// This assumes that the clocks are already configured properly. To configure
    /// them, you can either call [Radio::configure_clocks] or do the following:
    /// * enable the HFXO clock and wait until it is ready
    /// * set HFXO as the sysclk source
    pub fn new(mut radio_config: RadioConfig) -> RailResult<Self> {
        // The config if from the code example at https://docs.silabs.com/rail/latest/rail-api/
        let rail_config = unsafe {
            sl_rail_config {
                events_callback: Some(events_callback),
                p_opaque_handle1: &mut radio_config as *mut _ as *mut c_void,
                p_opaque_handle2: core::ptr::null_mut(),
                opaque_value: [0],
                rx_packet_queue_entries: SL_RAIL_BUILTIN_RX_PACKET_QUEUE_ENTRIES as u16,
                rx_fifo_bytes: SL_RAIL_BUILTIN_RX_FIFO_BYTES as u16,
                tx_fifo_bytes: radio_config.packet_length_bytes,
                tx_fifo_init_bytes: 0,
                p_rx_packet_queue: sl_rail_builtin_rx_packet_queue_ptr,
                p_rx_fifo_buffer: sl_rail_builtin_rx_fifo_ptr,
                p_tx_fifo_buffer: &mut FIFO_BUFFER[0],
            }
        };
        let rail_handle = Self::init(rail_config, radio_config.tx_power_dbm)?;

        Ok(Self {
            rail_handle,
            radio_config,
        })
    }

    /// Prepare for sending packets and start listening for packets.
    fn init(rail_config: sl_rail_config_t, tx_power_dbm: i16) -> RailResult<sl_rail_handle_t> {
        // This is ported from the code example at https://docs.silabs.com/rail/latest/rail-api/
        unsafe {
            sl_rail_util_pa_init();

            // https://github.com/SiliconLabs/simplicity_sdk/blob/b41bec3ff2485199c1a5a9995b3e649e118c1b8d/platform/radio/rail_lib/common/sl_rail_types.h#L119
            let mut rail_handle = 0xFFFF_FFFF as *mut c_void;
            sl_rail_init(
                (&mut rail_handle) as *mut *mut c_void,
                &rail_config,
                Some(init_callback),
            )
            .into_rail_result()?;

            sl_rail_config_cal(rail_handle, SL_RAIL_CAL_ALL).into_rail_result()?;

            let rail_config = &Protocol_Configuration_channelConfig as *const _
                as *const sl_rail_channel_config_t;
            // Taken from the `sl_rail_channel_config` type docs and from `radio_settings.radioconf` and `rail_config.c` of the `rail_soc_simple_trx` example project
            sl_rail_config_channels(
                rail_handle,
                rail_config,
                Some(sl_rail_util_pa_on_channel_config_change),
            )
            .into_rail_result()?;

            // configure the amount of power to use for sending packets
            // sl_rail_set_tx_power_dbm(rail_handle, 20 /* 20 dBm */).into_rail_result()?;
            // we can't use the above because its a macro that doesn't get auto-generated by bindgen
            RAIL_SetTxPowerDbm(rail_handle, tx_power_dbm).into_rail_result()?;

            // Configure the most useful callbacks and catch a few errors.
            sl_rail_config_events(
                rail_handle,
                SL_RAIL_EVENTS_ALL as u64,
                (SL_RAIL_EVENT_TX_PACKET_SENT
                    | SL_RAIL_EVENT_RX_PACKET_RECEIVED
                    | SL_RAIL_EVENT_RX_FRAME_ERROR) as u64,
            )
            .into_rail_result()?;

            // automatically transition back to rx mode after sending/receiving a packet
            let state_transitions = sl_rail_state_transitions {
                success: SL_RAIL_RF_STATE_RX as u8,
                error: SL_RAIL_RF_STATE_RX as u8,
            };
            sl_rail_set_rx_transitions(rail_handle, &state_transitions).into_rail_result()?;
            sl_rail_set_tx_transitions(rail_handle, &state_transitions).into_rail_result()?;

            // start listening for packets
            let channel = sl_rail_get_first_channel(rail_handle, rail_config);
            sl_rail_start_rx(rail_handle, channel, core::ptr::null());

            Ok(rail_handle)
        }
    }

    /// Send a packet.
    pub fn send_packet(&self, packet: &[u8]) -> RailResult<()> {
        unsafe {
            let packet_size_bytes = self.radio_config.packet_length_bytes;
            assert_eq!(packet.len() as u16, packet_size_bytes);

            // prepare packet
            // https://github.com/SiliconLabs/simplicity_sdk/blob/b41bec3ff2485199c1a5a9995b3e649e118c1b8d/app/rail/component/sl_rail_sdk_packet_assistant/sl_rail_sdk_packet_assistant.c#L594
            let bytes_written =
                sl_rail_write_tx_fifo(self.rail_handle, &packet[0], packet_size_bytes, true);
            if bytes_written != packet_size_bytes {
                return Err(RailError::TxFifoWriteFail(bytes_written, packet_size_bytes));
            }

            // channel ID is channel number start: https://github.com/SiliconLabs/simplicity_sdk/blob/b41bec3ff2485199c1a5a9995b3e649e118c1b8d/app/rail/component/sl_rail_sdk_phy_selector/sl_rail_sdk_phy_selector.c#L84
            let channel = sl_rail_get_first_channel(self.rail_handle, core::ptr::null());
            sl_rail_start_tx(
                self.rail_handle,
                channel,
                SL_RAIL_TX_OPTIONS_DEFAULT,
                core::ptr::null(),
            )
            .into_rail_result()
        }
    }

    /// Keep the received packet inside the queue and don't automatically flush it.
    /// This ensures that you can still read the content of the received packet later,
    /// and don't have to read it immediately when it arrives.
    fn hold_packet(&self) {
        unsafe { sl_rail_hold_rx_packet(self.rail_handle) };
    }

    /// Call the user-provided callback when a packet gets received.
    fn trigger_receive_callback(&self) {
        (self.radio_config.on_packet_received)();
    }

    /// Read a received packet.
    pub fn read_received_packet(&self, target_buffer: &mut [u8]) -> RailResult<()> {
        // will be overriden by sl_rail_get_rx_packet_info, so content doesn't matter
        let mut packet_info = sl_rail_rx_packet_info {
            packet_status: 0,
            packet_bytes: 0,
            first_portion_bytes: 0,
            p_first_portion_data: core::ptr::null_mut(),
            p_last_portion_data: core::ptr::null_mut(),
            filter_mask: 0,
        };
        // ported from official Simplicity SDK TRX example
        unsafe {
            let packet_handle = sl_rail_get_rx_packet_info(
                self.rail_handle,
                // https://github.com/SiliconLabs/simplicity_sdk/blob/sisdk-2025.6/platform/radio/rail_lib/common/rail_types.h#L4190 SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE,
                2 as RAIL_RxPacketHandle_t,
                &mut packet_info,
            );
            if packet_handle.is_null() {
                return Err(RailError::PacketBufferEmpty);
            }

            sl_rail_copy_rx_packet(self.rail_handle, &mut target_buffer[0], &packet_info)
                .into_rail_result()?;

            sl_rail_release_rx_packet(self.rail_handle, packet_handle).into_rail_result()?;
        }

        Ok(())
    }
}

// forward radio interrupts to RAIL
// these are all interrupt handlers defined in the RAIL blob
// inspect with `nm librail_efr32xg22_gcc_release.a`
#[interrupt]
fn RFSENSE() {
    unsafe {
        RFSENSE_IRQHandler();
    }
}
#[interrupt]
fn PRORTC() {
    unsafe {
        PRORTC_IRQHandler();
    }
}
#[interrupt]
fn AGC() {
    unsafe {
        AGC_IRQHandler();
    }
}
#[interrupt]
fn BUFC() {
    unsafe {
        BUFC_IRQHandler();
    }
}
#[interrupt]
fn EMUDG() {
    unsafe {
        EMUDG_IRQHandler();
    }
}
#[interrupt]
fn FRC() {
    unsafe {
        FRC_IRQHandler();
    }
}
#[interrupt]
fn FRC_PRI() {
    unsafe {
        FRC_PRI_IRQHandler();
    }
}
#[interrupt]
fn MODEM() {
    unsafe {
        MODEM_IRQHandler();
    }
}
#[interrupt]
fn PROTIMER() {
    unsafe {
        PROTIMER_IRQHandler();
    }
}
#[interrupt]
fn RAC_RSM() {
    unsafe {
        RAC_RSM_IRQHandler();
    }
}
#[interrupt]
fn RAC_SEQ() {
    unsafe {
        RAC_SEQ_IRQHandler();
    }
}
#[interrupt]
fn RDMAILBOX() {
    unsafe {
        RDMAILBOX_IRQHandler();
    }
}
#[interrupt]
fn SYNTH() {
    unsafe {
        SYNTH_IRQHandler();
    }
}

// We intercept the internal RAIL method that handles errors here, so that we can read the error code once
// RAIL crashes.
// In order to do this, RAILCb_AssertFailed has to be removed from the header file `rail.h`, because otherwise
// bindgen also generates a method for it, so it exists twice.
#[unsafe(no_mangle)]
pub extern "C" fn RAILCb_AssertFailed(_rail_handle: *mut c_void, error_code: u32) {
    // pub extern "C" fn sl_railcb_assert_failed(rail_handle: *mut c_void, error_code: u32, line: i32) {
    #[cfg(feature = "defmt-logging")]
    defmt::info!("rail crashed with code {}", error_code);
    panic!()
}
