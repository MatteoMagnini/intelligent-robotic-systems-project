-- Put your global variables here.

MOVE_STEPS = 5
MAX_VELOCITY = 5

FILENAME_PT = "../data/Qtable-phototaxis.csv"
FILENAME_OA = "../data/Qtable-obstacle-avoidance.csv"
WHEEL_DIST = robot.wheels.axis_length
n_steps = 0


--- This function is executed every time you press the 'execute' button.
function init()

  Qlearning = require "Qlearning"
  
  light_position = {1.5, 0.0, 0.5}
  robot_position = {}
  
  threshold = 0.0
  thresholds = {0.05, 0.33, 0.66, 1.0}
  
  -- States: each direction can be 0 or 1, a state is a combination of those.
  sensor_direction_names = {"NNW", "WNW", "WSW", "SSW", "SSE", "ESE", "ENE", "NNE"}
  
  -- So for oa in total the states are 2^8 = 256.
  number_of_states_oa = math.pow(2, #sensor_direction_names)
  
  -- So for pt in total the states are 4^8 = 65536.
  number_of_states_pt = math.pow(2, #sensor_direction_names)
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
  
  -- Dimension: 256 x 5 = 1280 values.
  Q_table_oa = Qlearning.load_Q_table(FILENAME_OA)
  
  -- Dimension: 65536 x 5 = 327680 values.
  Q_table_pt = Qlearning.load_Q_table(FILENAME_PT)
  
  local left_v = robot.random.uniform(0,MAX_VELOCITY)
	local right_v = robot.random.uniform(0,MAX_VELOCITY)
	robot.wheels.set_velocity(left_v,right_v)
  
end

-- Random walk
function competence0()
  --By default move randomly
  local left_v = robot.random.uniform(0,MAX_VELOCITY)
  local right_v = robot.random.uniform(0,MAX_VELOCITY)
  
  return left_v, right_v
  
end

-- Phototaxis
function competence1()

  function get_state_pt()
    
    -- State goes from 1 to 65536.
    -- 1 equals that all sensors are 1.
    local new_state = 1
    local max_intensity = 0
    
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

  local state = get_state_pt()
  local action = Qlearning.get_best_action(state, Q_table_pt)
  local subsumption = true
  
  if state == 1 then subsumption = false end
  
  return subsumption, action_to_velocity(action)

end

-- Obstacle Avoidance
function competence2()

  function get_state_oa()
    
    -- State goes from 1 to 256.
    -- 1 equals that all sensors are 1.
    local new_state = 1
    
    for i = 1, #sectors do
      for j = 1, #sectors[i] do
        if robot.proximity[sectors[i][j]].value > threshold then
          new_state = new_state + math.pow(2,i-1)
          break;
        end
      end
    end
    
    return new_state
    
  end

  local state = get_state_oa()
  local action = Qlearning.get_best_action(state, Q_table_oa)
  local subsumption = true
  
  if state == 1 then subsumption = false end
  local left_v, right_v = action_to_velocity(action)
  
  --Adding a little bit of noise to avoide stall
  return subsumption, left_v * robot.random.uniform(0.9,1), right_v * robot.random.uniform(0.9,1)

end

-- Map the index of the action into values for wheels.
function action_to_velocity(action)
  
  -- Ensure not to exceed MAX_VELOCITY
  function limit_v(left_v, right_v)

    if (left_v > MAX_VELOCITY) then
      left_v = MAX_VELOCITY
    end
    
    if (right_v > MAX_VELOCITY) then
      right_v = MAX_VELOCITY
    end

  end
  
  local angle = velocity_directions[velocity_direction_names[action]]
  local left_v = MAX_VELOCITY - (angle * WHEEL_DIST / 2)
  local right_v = MAX_VELOCITY + (angle * WHEEL_DIST / 2)
  
  limit_v(left_v, right_v)  
  
  return left_v, right_v
  
end


--- This function is executed at each time step.
-- It must contain the logic of your controller.
function step()
	n_steps = n_steps + 1
  
  -- Perform action
  if n_steps % MOVE_STEPS == 0 then
  
    left_v0, right_v0 = competence0()
    subsumption1, left_v1, right_v1 = competence1()
    subsumption2, left_v2, right_v2 = competence2()
    
    if (subsumption2) then
      robot.wheels.set_velocity(left_v2,right_v2)
    else
      if (subsumption1) then
        robot.wheels.set_velocity(left_v1,right_v1)
      else
        robot.wheels.set_velocity(left_v0,right_v0)
      end
    end
    
    robot_position = {robot.positioning.position.x, robot.positioning.position.y, robot.positioning.position.z}

	end
 
end


--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	local left_v = 0
	local right_v = 0
	robot.wheels.set_velocity(left_v,right_v)
	n_steps = 0
end


--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
  -- calculating the metric.
  euclidean_distance = math.sqrt(math.pow(light_position[1] - robot_position[1], 2) +
    math.pow(light_position[2] - robot_position[2], 2) +
    math.pow(light_position[3] - robot_position[3], 2))
  euclidean_distance = math.floor((euclidean_distance) * 10000) / 10000
    
  print(euclidean_distance .. ", !!marker!!")
   
end
