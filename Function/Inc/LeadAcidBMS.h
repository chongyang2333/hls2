#ifndef _LEADACIDBMS_H_
#define _LEADACIDBMS_H_

#include  "UserDataTypes.h"
#include "LeadAcidBMS.h"


/**
 * @brief 铅蓄电池BMS事件
 */
typedef enum 
{
    LEAD_ACID_EVENT_NONE = 0,
    LEAD_ACID_EVENT_INSERT_IN,
    LEAD_ACID_EVENT_CHARGING,
    LEAD_ACID_EVENT_PULL_OUT
} LeadAcidEvent_t;


PUBLIC void LeadAcidBatteryInit(void);
// PUBLIC float GetBatterySoc(void);
PUBLIC void LeadAcidInfoUpdate(void);
PUBLIC void LeadAcideEventTask(LeadAcidEvent_t event);
PUBLIC void lead_acid_battery_info(void);
PUBLIC void LeadAcidBatteryDischargeListen(void);
PUBLIC void LeadAcidBatteryChargeListen(void);

#endif
