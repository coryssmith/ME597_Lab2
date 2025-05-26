from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat

    # Compute yaw (theta) only
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return yaw  # Return only yaw, as per Lab 1

#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    current_x, current_y = current_pose[0], current_pose[1]
    goal_x, goal_y = goal_pose[0], goal_pose[1]

    # Remember to use the Euclidean distance to calculate the error.
    error_linear= sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    current_x, current_y, current_theta = current_pose[0], current_pose[1], current_pose[2]
    goal_x = goal_pose[0]
    goal_y = goal_pose[1]

    # Use atan2 to find the desired orientation
    # Desired orientation (theta_goal)
    theta_goal = atan2(goal_y - current_y, goal_x - current_x)

    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal
    # Angular error
    error_angular = theta_goal - current_theta

    # Remember to handle the cases where the angular error might exceed the range [-π, π]
    # Normalize error to range [-π, π]
    while error_angular > M_PI:
        error_angular -= 2 * M_PI
    while error_angular < -M_PI:
        error_angular += 2 * M_PI
    
    return error_angular
