#include "EventSource.h"

using namespace ESP32;

#if CH_CFG_USE_EVENTS == TRUE

bool EventSource::wait(uint16_t duration_us, AP_HAL::EventHandle *evt_handle)
{
    return true;
}

void EventSource::signal(uint32_t evt_mask)
{
}

__RAMFUNC__ void EventSource::signalI(uint32_t evt_mask)
{
}
#endif //#if CH_CFG_USE_EVENTS == TRUE
