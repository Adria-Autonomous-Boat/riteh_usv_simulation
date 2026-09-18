#include <behaviortree_cpp/bt_factory.h>
#include "usv_bt/gps_anchor_ready_condition.hpp"
#include "usv_bt/navigate_channel_action.hpp"
#include "usv_bt/vessel_in_range_condition.hpp"
#include "usv_bt/colreg_avoid_action.hpp"
#include "usv_bt/vessel_clear_condition.hpp"
#include "usv_bt/northing_above_condition.hpp"

BT_REGISTER_NODES (factory)
{
    factory.registerNodeType<usv_bt::GpsAnchorReady>("GpsAnchorReady");
    factory.registerNodeType<usv_bt::NavigateChannel>("NavigateChannel");
    factory.registerNodeType<usv_bt::VesselInRange>("VesselInRange");
    factory.registerNodeType<usv_bt::ColregAvoid>("ColregAvoid");
    factory.registerNodeType<usv_bt::VesselClear>("VesselClear");
    factory.registerNodeType<usv_bt::NorthingAbove>("NorthingAbove");

}
