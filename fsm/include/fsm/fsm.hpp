#ifndef FSM_HPP
#define FSM_HPP
#include <string>

enum class FSMState {
  IDLE,
  PLANNING,
  DRIVING,
  SLOW_DOWN,
  STOP,
  WAIT,
  PEDESTRIAN_WAIT,
  YIELD,
  REPLANNING,
  APPROACHING,
  PASSENGER_PICKUP,
  PASSENGER_DROPOFF,
  LANE_CHANGE,
  TUNNEL,
  PARKING,
  ARRIVED,
  EMERGENCY
};

inline std::string stateToString(FSMState state)
{
  switch (state) {
    case FSMState::IDLE:             return "IDLE";
    case FSMState::PLANNING:         return "PLANNING";
    case FSMState::DRIVING:          return "DRIVING";
    case FSMState::SLOW_DOWN:        return "SLOW_DOWN";
    case FSMState::STOP:             return "STOP";
    case FSMState::WAIT:             return "WAIT";
    case FSMState::PEDESTRIAN_WAIT:  return "PEDESTRIAN_WAIT";
    case FSMState::YIELD:            return "YIELD";
    case FSMState::REPLANNING:       return "REPLANNING";
    case FSMState::APPROACHING:      return "APPROACHING";
    case FSMState::PASSENGER_PICKUP: return "PASSENGER_PICKUP";
    case FSMState::PASSENGER_DROPOFF:return "PASSENGER_DROPOFF";
    case FSMState::LANE_CHANGE:      return "LANE_CHANGE";
    case FSMState::TUNNEL:           return "TUNNEL";
    case FSMState::PARKING:          return "PARKING";
    case FSMState::ARRIVED:          return "ARRIVED";
    case FSMState::EMERGENCY:        return "EMERGENCY";
    default:                         return "UNKNOWN";
  }
}

#endif
