-- Put your global variables here.

MOVE_STEPS = 5
MAX_VELOCITY = 5
FILENAME = "../data/Qtable-phototaxis2.csv"
WHEEL_DIST = robot.wheels.axis_length
n_steps = 0


--- This function is executed every time you press the 'execute' button.
function init()

  Qlearning = require "Qlearning"
  
	-- First action is to go forward.
	local left_v = MAX_VELOCITY
	local right_v = MAX_VELOCITY
  
  alpha = 0.1
  gamma = 0.9
  epsilon = 0.9
  k = 2
  thresholds = {0.05, 0.33, 0.66, 1.0}
  
  old_state = get_state()
  state = old_state
  action = 3
  
  -- States: each direction can be 0,1,2 or 3 (dark, low, medium, high intensity) a state is a combination of those.
  -- So in total the states are 4^8 = 65536.
  sensor_direction_names = {"NNW", "WNW", "WSW", "SSW", "SSE", "ESE", "ENE", "NNE"}
  number_of_states = math.pow(4, #sensor_direction_names)
  sectors = {
    {1,2,3},
    {4,5,5},
    {7,8,9},
    {10,11,12},
    {13,14,15},
    {16,17,18},
    {19,20,21},
    {22,23,24},
  }
  
  -- Actions: 5 in total
  -- Threre is no symmetry: a vector direction cannot be cancelled by a opposite vector.
  -- In this way we avoid stupid behaviours such that go forward and then immidiatly backward.
  velocity_direction_names = {"WNW", "NNW", "N", "NNE", "ENE"}
  velocity_directions = {
    ["WNW"] = math.pi / 4, -- 45 degree
    ["NNW"] = math.pi / 8, -- 22.5 degree
    ["N"] = 0,
    ["NNE"] = - math.pi / 8, -- -22.5 degree
    ["ENE"] = - math.pi / 4, -- 45 degree
  }
  
  number_of_actions = #velocity_direction_names
  
  Q_table = {}
  
  -- Dimension: 256 x 5 = 1280 values.
  Q_table = Qlearning.load_Q_table(FILENAME)
  
	robot.wheels.set_velocity(left_v,right_v)
  
end

function get_state()
  
  -- State goes from 1 to 256.
  -- 1 equals that all sensors are 1.
  local new_state = 1
  
  for i = 1, #sectors do
    local intensity = 0
    for j = 1, #sectors[i] do
      if robot.light[sectors[i][j]].value > intensity then
        intensity = robot.light[sectors[i][j]].value      
      end
    end
    for t = 1, #thresholds do
      if intensity < thresholds[t] then
        new_state = new_state + (math.pow(4,i-1) * (t - 1))
        break
      end
    end
  end
  
  return new_state
  
end

function get_reward()

  local points = 0
  
  for i = 1, #sectors do
    for j = 1, #sectors[i] do
      local intensity = 0
      for j = 1, #sectors[i] do
        if robot.light[sectors[i][j]].value > intensity then
          intensity = robot.light[sectors[i][j]].value      
        end
      end
      for t = 1, #thresholds do
        if intensity < thresholds[t] then
          points = points + t - 1
          break
        end
      end
    end
  end
  
  return math.pow(points/(#sectors * (#thresholds - 1)), 2)

end

function perform_action(action)

  -- Ensure not to exceed MAX_VELOCITY
  function limit_v(left_v, right_v)

    function limit(value)
      if (value > MAX_VELOCITY) then
        value = MAX_VELOCITY
      end
      
      if (value < - MAX_VELOCITY) then
        value = - MAX_VELOCITY
      end

      return value
    end
    
    return limit(left_v), limit(right_v)

  end
  
  local angle = velocity_directions[velocity_direction_names[action]]
  local left_v = MAX_VELOCITY - (angle * WHEEL_DIST / 2)
  local right_v = MAX_VELOCITY + (angle * WHEEL_DIST / 2)
  
  left_v, right_v = limit_v(left_v,right_v)
  
  robot.wheels.set_velocity(left_v,right_v)
  
end

--- This function is executed at each time step.
-- It must contain the logic of your controller.
function step()
	n_steps = n_steps + 1

	if n_steps % MOVE_STEPS == 0 then
      
    -- Update
    state = get_state()
    Q_table = Qlearning.update_Q_table(alpha, gamma, old_state, action, get_reward(), state, Q_table)
  
    -- Perform action
    old_state = state
    action = Qlearning.get_random_action(epsilon, old_state, Q_table)
    --action = Qlearning.get_weighted_action(k, old_state, Q_table)
    perform_action(action)

	end
 
end



--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	local left_v = MAX_VELOCITY
	local right_v = MAX_VELOCITY
  action = 3
	robot.wheels.set_velocity(left_v,right_v)
	n_steps = 0
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
   
   Qlearning.save_Q_table(FILENAME, Q_table)
   
end
