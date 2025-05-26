from math import atan2, asin, sqrt
import csv
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
            #csv_writer = csv.writer(file)
            #csv_writer.writerow(headers)

    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            # TODO Part 5: Write the values from the list to the file
            # The following commented DID NOT WORK FOR SOME REASON 
            # vals_str = ", ".join(map(str, values_list))  # Convert list to comma-separated string 
            #vals_str+="\n"
            #file.write(vals_str)

            #Use CSV method 
            writer = csv.writer(file)  # Create CSV writer object
            writer.writerow(values_list)  # Write list as a row
            
            
            

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
            # Skip the header line

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


# TODO Part 5: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    # yaw is rotation about z-axis
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat.x, quat.y, quat.z, quat.w  # Extract quaternion components
    
    # Compute Roll (φ) - Rotation around X-axis
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    
    # Compute Pitch (θ) - Rotation around Y-axis (Clamp to avoid errors)
    sin_pitch = 2.0 * (w * y - x * z)
    pitch = asin(max(-1.0, min(1.0, sin_pitch)))  # Clamped between -1 and 1
    
    # Compute Yaw (ψ) - Rotation around Z-axis
    yaw = atan2(2.0 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    ... # just unpack yaw
    return yaw


