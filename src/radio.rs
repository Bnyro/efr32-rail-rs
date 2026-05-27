use core::ffi::c_void;

use efr32mg22_pac::{Peripherals, interrupt};

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

pub struct Radio {
    rail_handle: sl_rail_handle_t,
    packet_length_bytes: u16,
}

impl From<sl_rail_handle_t> for Radio {
    fn from(rail_handle: sl_rail_handle_t) -> Self {
        let rail_config = unsafe { sl_rail_get_config(rail_handle) };
        Radio {
            rail_handle,
            packet_length_bytes: unsafe { (*rail_config).tx_fifo_bytes },
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
    pub fn new(packet_length_bytes: u16, on_packet_received: fn()) -> Self {
        // The config if from the code example at https://docs.silabs.com/rail/latest/rail-api/
        let rail_config = unsafe {
            sl_rail_config {
                events_callback: Some(events_callback),
                p_opaque_handle1: on_packet_received as *mut c_void,
                p_opaque_handle2: core::ptr::null_mut(),
                opaque_value: [0],
                rx_packet_queue_entries: SL_RAIL_BUILTIN_RX_PACKET_QUEUE_ENTRIES as u16,
                rx_fifo_bytes: SL_RAIL_BUILTIN_RX_FIFO_BYTES as u16,
                tx_fifo_bytes: packet_length_bytes,
                tx_fifo_init_bytes: 0,
                p_rx_packet_queue: sl_rail_builtin_rx_packet_queue_ptr,
                p_rx_fifo_buffer: sl_rail_builtin_rx_fifo_ptr,
                p_tx_fifo_buffer: &mut FIFO_BUFFER[0],
            }
        };
        let rail_handle = Self::init(rail_config);

        Self {
            rail_handle,
            packet_length_bytes,
        }
    }

    /// Prepare for sending packets and start listening for packets.
    fn init(p_rail_config: sl_rail_config_t) -> sl_rail_handle_t {
        // This is ported from the code example at https://docs.silabs.com/rail/latest/rail-api/
        unsafe {
            sl_rail_util_pa_init();

            // https://github.com/SiliconLabs/simplicity_sdk/blob/b41bec3ff2485199c1a5a9995b3e649e118c1b8d/platform/radio/rail_lib/common/sl_rail_types.h#L119
            let mut p_rail_handle = 0xFFFF_FFFF as *mut c_void;
            let status = sl_rail_init(
                (&mut p_rail_handle) as *mut *mut c_void,
                &p_rail_config,
                Some(init_callback),
            );
            assert_eq!(status, RAIL_STATUS_NO_ERROR);

            let status = sl_rail_config_cal(p_rail_handle, SL_RAIL_CAL_ALL);
            assert_eq!(status, RAIL_STATUS_NO_ERROR);

            // Taken from the `sl_rail_channel_config` type docs and from `radio_settings.radioconf` and `rail_config.c` of the `rail_soc_simple_trx` example project
            let status = sl_rail_config_channels(
                p_rail_handle,
                &Protocol_Configuration_channelConfig as *const _
                    as *const sl_rail_channel_config_t,
                Some(sl_rail_util_pa_on_channel_config_change),
            );
            assert_eq!(status, RAIL_STATUS_NO_ERROR);

            // let status = sl_rail_set_tx_power_dbm(p_rail_handle, 20 /* 20 dBm */);
            // we can't use the above because its a macro that doesn't get auto-generated by bindgen
            let status = RAIL_SetTxPowerDbm(p_rail_handle, 20 /* 20 dBm */);
            assert_eq!(status, RAIL_STATUS_NO_ERROR);

            // Configure the most useful callbacks and catch a few errors.
            let status = sl_rail_config_events(
                p_rail_handle,
                SL_RAIL_EVENTS_ALL as u64,
                (SL_RAIL_EVENT_TX_PACKET_SENT
                    | SL_RAIL_EVENT_RX_PACKET_RECEIVED
                    | SL_RAIL_EVENT_RX_FRAME_ERROR) as u64,
            );
            assert_eq!(status, RAIL_STATUS_NO_ERROR);

            // Set automatic transitions to always receive once started.
            let p_state_transitions = sl_rail_state_transitions {
                success: SL_RAIL_RF_STATE_RX as u8,
                error: SL_RAIL_RF_STATE_RX as u8,
            };
            let status = sl_rail_set_rx_transitions(p_rail_handle, &p_state_transitions);
            assert_eq!(status, RAIL_STATUS_NO_ERROR);
            let status = sl_rail_set_tx_transitions(p_rail_handle, &p_state_transitions);
            assert_eq!(status, RAIL_STATUS_NO_ERROR);

            p_rail_handle
        }
    }

    /// Send a packet.
    pub fn send_packet(&self, packet: &[u8]) {
        unsafe {
            // prepare packet
            // https://github.com/SiliconLabs/simplicity_sdk/blob/b41bec3ff2485199c1a5a9995b3e649e118c1b8d/app/rail/component/sl_rail_sdk_packet_assistant/sl_rail_sdk_packet_assistant.c#L594
            let bytes_written =
                sl_rail_write_tx_fifo(self.rail_handle, &packet[0], self.packet_length_bytes, true);
            assert_eq!(bytes_written, self.packet_length_bytes as u16);

            // channel ID is channel number start: https://github.com/SiliconLabs/simplicity_sdk/blob/b41bec3ff2485199c1a5a9995b3e649e118c1b8d/app/rail/component/sl_rail_sdk_phy_selector/sl_rail_sdk_phy_selector.c#L84
            let channel = sl_rail_get_first_channel(self.rail_handle, core::ptr::null());
            let status = sl_rail_start_tx(
                self.rail_handle,
                channel,
                SL_RAIL_TX_OPTIONS_DEFAULT,
                core::ptr::null(),
            );
            assert_eq!(status, SL_RAIL_STATUS_NO_ERROR);
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
        let rail_config = unsafe { sl_rail_get_config(self.rail_handle) };
        let receive_callback: fn() =
            unsafe { core::mem::transmute((*rail_config).p_opaque_handle1) };
        (receive_callback)();
    }

    /// Read a received packet.
    pub fn read_received_packet(&self, target_buffer: &mut [u8]) {
        // will be overriden by sl_rail_get_rx_packet_info, so content doesn't matter
        let mut p_packet_info = sl_rail_rx_packet_info {
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
                &mut p_packet_info,
            );
            assert!(!packet_handle.is_null());

            let status =
                sl_rail_copy_rx_packet(self.rail_handle, &mut target_buffer[0], &p_packet_info);
            assert_eq!(status, SL_RAIL_STATUS_NO_ERROR);

            let status = sl_rail_release_rx_packet(self.rail_handle, packet_handle);
            assert_eq!(status, SL_RAIL_STATUS_NO_ERROR);
        }
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
