cmake_minimum_required(VERSION 3.5)

set(TINUSB_DIR ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src)

function(add_tinyusb TARGET)

  set(TINYUSB_SRC
        ${TINUSB_DIR}/tusb.c
        ${TINUSB_DIR}/common/tusb_fifo.c
        # device
        ${TINUSB_DIR}/device/usbd.c
        ${TINUSB_DIR}/device/usbd_control.c
        ${TINUSB_DIR}/class/audio/audio_device.c
        ${TINUSB_DIR}/class/cdc/cdc_device.c
        ${TINUSB_DIR}/class/dfu/dfu_device.c
        ${TINUSB_DIR}/class/dfu/dfu_rt_device.c
        ${TINUSB_DIR}/class/hid/hid_device.c
        ${TINUSB_DIR}/class/midi/midi_device.c
        ${TINUSB_DIR}/class/msc/msc_device.c
        ${TINUSB_DIR}/class/net/ecm_rndis_device.c
        ${TINUSB_DIR}/class/net/ncm_device.c
        ${TINUSB_DIR}/class/usbtmc/usbtmc_device.c
        ${TINUSB_DIR}/class/vendor/vendor_device.c
        ${TINUSB_DIR}/class/video/video_device.c
        # host
        ${TINUSB_DIR}/host/usbh.c
        ${TINUSB_DIR}/host/hub.c
        ${TINUSB_DIR}/class/cdc/cdc_host.c
        ${TINUSB_DIR}/class/hid/hid_host.c
        ${TINUSB_DIR}/class/msc/msc_host.c
        ${TINUSB_DIR}/class/vendor/vendor_host.c
        # typec
        ${TINUSB_DIR}/typec/usbc.c
#        ${TINUSB_DIR}/portable/wch/dcd_ch32_usbhs.c
  )

  message ( "TINYUSB LIB:${TINYUSB_SRC}" )

  target_sources(${TARGET} PRIVATE
    # common
          ${TINYUSB_SRC}
  )
  target_include_directories(${TARGET} PUBLIC
    ${TINUSB_DIR}
    ${TINUSB_DIR}/portable/wch
    ${TINUSB_DIR}/../hw
    # TODO for net driver, should be removed/changed
    ${TINUSB_DIR}/../lib/networking
  )

endfunction()
