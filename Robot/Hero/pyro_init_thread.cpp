#include "pyro_bsp_can.h"
#include "pyro_bsp_uart.h"
#include "pyro_can_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_image_drv.h"
#include "pyro_ins.h"
#include "pyro_supercap_drv.h"
#include "pyro_referee.h"
#include "pyro_sr04_drv.h"
#include "pyro_sr05_drv.h"
#include "pyro_us100_drv.h"
#include "pyro_vt03_rc_drv.h"

namespace pyro
{
extern "C"
{
    can_drv_t *can1_drv = nullptr;
    can_drv_t *can2_drv = nullptr;
    can_drv_t *can3_drv = nullptr;
    ins_drv_t *ins_drv  = nullptr;

    void pyro_init_thread(void *argument)
    {
        dwt_drv_t::init(480); // Initialize DWT at 480 MHz

        // 初始化所有 CAN
        bsp_can::init_all();

        // （可选）保存指针供外部使用（如果需要）
        can1_drv = &bsp_can::get_can1();
        can2_drv = &bsp_can::get_can2();
        can3_drv = &bsp_can::get_can3();

        // 初始化 INS
        ins_drv = ins_drv_t::get_instance();
        ins_config_t config{};
        config.direct = ins_config_t::imu_direct_t::DIRECT_1;  // 根据实际安装方向调整
        config.calibrate = false;                              // 首次使用可设为 true 校准
        config.gx_offset = 0.0f;
        config.gy_offset = 0.0f;
        config.gz_offset = 0.0f;
        config.g_norm = 9.80665f;
        ins_drv->init(config);

#ifdef DR16_UART
        dr16_drv_t::instance().start();
        dr16_drv_t::instance().enable();
        DR16_UART.reset(100000, UART_WORDLENGTH_9B, UART_STOPBITS_2,
                        UART_PARITY_EVEN);
        DR16_UART.enable_rx_dma();
#endif

#ifdef VT03_UART
        vt03_drv_t::instance().start();
        vt03_drv_t::instance().enable();
        VT03_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
        VT03_UART.enable_rx_dma();
        rc_drv_t::init_virtual_rc();
#endif


#ifdef REFEREE_UART
        REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        REFEREE_UART.enable_rx_dma();
        referee_drv_t::get_instance()->init();
#endif

#ifdef SUPERCAP_UART
        SUPERCAP_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                            UART_PARITY_NONE);
        SUPERCAP_UART.enable_rx_dma();
        supercap_drv_t::get_instance()->start_rx();
#endif

#ifdef AUTOAIM_UART
        AUTOAIM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        AUTOAIM_UART.enable_rx_dma();
#endif

#ifdef IMAGE_UART
        IMAGE_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                         UART_PARITY_NONE);
        image_drv_t::get_instance().start();
        image_drv_t::get_instance().init();
#endif

#ifdef CUSTOM_UART
        CUSTOM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                          UART_PARITY_NONE);
#endif

#ifdef SR04_UART
        SR04_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
        SR04_UART.enable_rx_dma();
        sr04_drv::get_instance().init();
        sr04_drv::get_instance().start();
#endif

#ifdef US100_UART
        US100_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                         UART_PARITY_NONE);
        us100_drv::get_instance().init();
        us100_drv::get_instance().start();
#endif

#ifdef SR05_UART
        SR05_UART.set_level_invert(false,true);
        SR05_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                                UART_PARITY_NONE);
        SR05_UART.enable_rx_dma();
        sr05_drv::get_instance().init();
        sr05_drv::get_instance().start();
#endif

        vTaskDelete(nullptr);
    }
}
} // namespace pyro