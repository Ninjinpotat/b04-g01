
# Importing libraries is easy in Python.  Just use the import statement.
# We only want to import specific functions from the lidar library that we are using
# so we use the from ... import ... statement.
# In this example, we are importing the BAUDRATE, lidarConnect, lidarDisconnect, process_scan, lidarStatus, startScan, performSingleScan, and printScanHistogram functions from the lidar.alex_lidar library.
from lidar.alex_lidar import  BAUDRATE, lidarConnect, lidarDisconnect, lidarStatus, performSingleScan, resampleLidarScan


PORT = "/dev/ttyUSB0"   # Please change this to the correct port for your device if necessary
BAUDRATE = 115200

# In python, we define functions using the def keyword. The function name is followed by parentheses and a colon.
# Python uses indentation to define the scope of the function. Everything indented under the function definition is part of the function.
# This is similar to the curly braces {} in C or Java.
def TestLidarSingleScan():
    """
    Main function to demonstrate a single scan using the LiDAR device.
    
    """
    # Print() is a function that prints text to the console
    print("====== LiDAR connection test ======") 
    # We imported the lidarConnect function from the lidar.alex_lidar library, so we can use it here
    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    print("Connected to LiDAR device on port", PORT) #Print() can take multiple arguments, separated by commas
    print("Baudrate:", BAUDRATE) #Print will try to convert arguments to strings if they are not strings
    print("") #Print a blank line

    print("====== Device information ======")
    print("Retrieving LiDAR device information...")

    # Python function can take in normal positional arguments and keyword arguments
    # Keyword arguments are specified by name=value
    # Here, we are passing the lidar object to the lidarStatus function as a positional argument
    # We are also passing the keyword argument verbose=True to make sure the function prints the status information
    status = lidarStatus(lidar, verbose=True) 

    # The lidarStatus function returns a dictionary of status information
    # dictionaries are key-value pairs. We can access the value using the dictionary[key] syntax
    scan_mode = status['typical_scan_mode'] 
    print("")


    print("====== Scanning ======")
    
    # f-strings are a way to format strings in Python
    # The f before the string tells Python to format the string
    # Curly braces {} are placeholders
    # Variables can be inserted into the string by putting the variable name inside the curly braces
    print(f"Starting a single scan on mode {scan_mode}...") 
    
    # Get a single scan using the library
    # Read the documentation for the performSingleScan function in the lidar.alex_lidar library
    # scanResults is a tuple containing the scan angles and distances
    scanResults = performSingleScan(lidar, scan_mode)

    # Process the scan data
    # Tuples and lists are orderd sequences of elements in Python. They can contain any type of data
    # Lists can be written as [item1, item2, item3, ...]
    # Tuples can be written as (item1, item2, item3, ...)
    # Lists are mutable, meaning they can be changed after creation
    # Tuples are immutable, meaning they cannot be changed after creation
    # In this case, scanResults is a tuple containing two lists: scanAngles and scanDistances
    # We can access the elements of a tuple/list using indexing, starting from 0
    scanAngles = scanResults[0]
    scanDistances = scanResults[1]

    # print the histogram of the scan data
    print(f"Scan OK!  Got {len(scanAngles)} scan points.")
    print("Scan data histogram:")
    printScanHistogram(scanDistances, scanAngles, rows=12, cols=120)
    
    # Disconnect from the LiDAR device
    print("Disconnecting from LiDAR device...")
    lidarDisconnect(lidar)


def printScanHistogram(distances, angles, rows = 10, cols=60 ):
    """
    Prints a histogram of the LIDAR scan data. 
    
    The histogram is displayed as a grid of characters, where each character represents a distance measurement. The grid is divided into rows and columns, with each row corresponding to a range of distances and each column corresponding to an angle. The maximum distance in the scan is used to determine the height of the grid, and the distances are normalized to fit within the specified number of rows. 
    
    The grid is then filled with characters based on the distances, with a different character used for distances above and below a certain threshold.

    Args:
        distances (list): A list of distance measurements from the LIDAR scan.
        angles (list): A list of angles corresponding to the distance measurements.
        rows (int, optional): The number of rows in the histogram grid. Defaults to 10.
        cols (int, optional): The number of columns in the histogram grid. Defaults to 60

    Returns:
        None
    """
    
    # Resample the LIDAR scan data to fit the specified number of columns
    # The resampleLidarScan function is defined in the lidar.alex_lidar library
    # It takes the distance and angle measurements from the LIDAR scan and resamples them to fit the specified number of columns
    # The resampled data is returned as a tuple of distances and angles --> (distances, angles)
    # distances and angles are lists of the same length, with each element corresponding to a distance and angle measurement, respectively
    distances, angles = resampleLidarScan(distance=distances, angles=angles, target_measurements_per_scan=cols)

    # max and min functions are used to find the maximum and minimum values in a list
    # The max function returns the largest value in the list
    # The min function returns the smallest value in the list
    max_distance = max(distances)
    symbolA = " "
    symbolB = "*"

    # Python List Comprehensions are a concise way to create lists
    # The syntax is [expression for item in list]
    # Here, we are creating a 2D list (matrix) using list comprehension, which also demonstrates that lists can be nested
    # [symbolA for i in range(cols)] creates a list of cols elements, each containing the symbolA character
    # [[symbolA for i in range(cols)] for j in range(rows)] creates a 2D list with rows rows and cols columns, filled with symbolA characters
    matrix = [[symbolA for i in range(cols)] for j in range(rows)]


    # Loops in Python are similar to loops in other languages
    # They follow the same syntax: for i in sequence:
    # The range function generates a sequence of numbers from 0 to n-1
    # The for loop then iterates over this sequence
    for i in range(rows):
        for j in range(cols):
            # What is happening here?
            if distances[j] > (max_distance/rows)*(rows-i):
                matrix[i][j] = symbolB
            else:
                matrix[i][j] = symbolA

    for i in range(rows):
        print("".join(matrix[i]))

    print("="*cols) # The * operator can be used to repeat a string multiple times
    print("0" + " "*int(cols-4) + "359") # The int function tries to convert a value to an integer

if __name__ == "__main__":
    TestLidarSingleScan()
