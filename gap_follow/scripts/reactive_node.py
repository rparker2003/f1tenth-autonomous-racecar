import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    This class implements a reactive method which drives towards the center of 
    a gap and turns into tight gaps without clipping walls or corners. 
    Following the Gap
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        self.laser_sub = self.create_subscription (
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.VELOCITY = 1.0 # speed
        self.MAX_ADMISSABLE_DIST = 3.0 # floor all greater values to this if scanned
        self.MIN_ADMISSABLE_DIST = 0.3 # anything X meters away from us is now not even considered for gaps
        self.CAR_RADIUS = .5 # width in radius to bubble out
        self.SAFE_WALL_DISTANCE = 0.1 # the safest distance to be from the wall

        # speed and angle comparison values declaration
        self.STRAIGHT_AHEAD_SPEED = 3.0
        self.STRAIGHT_AHEAD_THRESHOLD = 10*np.pi/180
        self.WIDE_TURN_SPEED = 1.5
        self.WIDE_TURN_THRESHOLD = 20*np.pi/180
        self.SHARP_TURN_SPEED = 0.75

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        for i, j in enumerate(ranges):
            if j > self.MAX_ADMISSABLE_DIST:
                ranges[i] = self.MAX_ADMISSABLE_DIST
        return ranges

    def find_max_gap(self, free_space_ranges, direct_right_index, direct_left_index):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        ranges = free_space_ranges

        max_gap_i, max_gap_j = direct_right_index, direct_right_index
        curr_gap_i, curr_gap_j = direct_right_index, direct_right_index
        inGap = False

        for i in range(direct_right_index, direct_left_index+1):
            if(inGap and ranges[i] <= self.MIN_ADMISSABLE_DIST):
                if(curr_gap_j-curr_gap_i > max_gap_j-max_gap_i):
                    max_gap_i, max_gap_j = curr_gap_i, curr_gap_j
                inGap = False
            elif(inGap and ranges[i] >= self.MIN_ADMISSABLE_DIST):
                curr_gap_j = i;
            elif(not inGap and ranges[i] <= self.MIN_ADMISSABLE_DIST):
                continue
            elif(not inGap and ranges[i] >= self.MIN_ADMISSABLE_DIST):
                curr_gap_i = i
                inGap = True

    
        if(curr_gap_j-curr_gap_i > max_gap_j-max_gap_i):
            max_gap_i, max_gap_j = curr_gap_i, curr_gap_j
            inGap = False
        
        return max_gap_i, max_gap_j

    def find_best_point(self, start_i, end_i, ranges):
        """ Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges

	    Naive: Choose the furthest point within ranges and go there
        """
        # start and end indices of max gap
        max_index_start = start_i 
        max_index_end = start_i
        for i in range(start_i, end_i+1):
            if ranges[i] >= ranges[max_index_start]:
                max_index_start = i
            if ranges[i] > ranges[max_index_end]:
                max_index_end = i
        # this returns the center of the gap between the farthest points
        # therefore centering the car in the hallway
        return int((max_index_start+max_index_end)/2)

    def bubble(self, ranges, direct_right_index, direct_left_index, angle_increment):
        """
        Find closest point to LiDAR
        Eliminate all points inside 'bubble' (set them to zero)
        """
        #Find closest point to LiDAR
        min_index = direct_right_index
        for i in range(direct_right_index, direct_left_index+1):
            if ranges[i] < ranges[min_index]:
                min_index = i
        
        # print(f"Closest Point Angle:\t{(min_index*angle_increment+angle_min)*180/np.pi}") 
        # get our angle by using arctan
        bubble_rads = abs(np.arctan(self.CAR_RADIUS/ranges[min_index]))
        # number of indices to 'bubble'
        bubble_indices = (int)(bubble_rads/angle_increment)


        #Eliminate all points inside 'bubble' (set them to zero) 
        for i in range(0, bubble_indices+1):
            currentBubbleRight = min_index+i 
            currentBubbleLeft = min_index-i

            if currentBubbleRight < len(ranges):
                ranges[currentBubbleRight] = 0 
            if currentBubbleLeft > 0:
                ranges[currentBubbleLeft] = 0
        
        return min_index

    def print_ranges_fancy(self, data):
        """ Takes data as param, returns nothing. Purpose is to print a nice 
        output of the radian and degree values at every index in ranges 
        for debugging purposes. Exits when it's done printing.
        """
        print("Index\tDegrees\tRads\tDistance")
        for i, j in enumerate(data.ranges):
            Rad = i * data.angle_increment + data.angle_min
            Deg = Rad * 180 / np.pi
            print(f"{i:.3f}\t{Deg:.3f}\t{Rad:.3f}\t{j:.3f}")
        exit()
    
    def indexToDeg(self, index, data):
        """ Takes an index and data as param, returns that index as a value in 
        degrees.
        """
        return (index*data.angle_increment+data.angle_min)*180/np.pi
        
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        self.preprocess_lidar(data.ranges)
        
        # indices that correlate to -90 (right) and 90 (left) degrees
        direct_right_index = int((np.pi/4)/data.angle_increment)
        direct_left_index = int(direct_right_index+np.pi/data.angle_increment)
        
        # TODO: Implement disparity and replace bubble. Fixes lookahead dist.
        # Call to the bubble function, it does every bubble task
        self.bubble(data.ranges, direct_right_index, direct_left_index, data.angle_increment)

        #Find the gap with max length
        maxGapStart_i, maxGapEnd_i = self.find_max_gap(data.ranges, direct_right_index, direct_left_index)
        #print(f"From {maxGapStart_i} to {maxGapEnd_i}")
        # self.print_ranges_fancy(data=data)

        #Find the best point in the gap 
        best_point = self.find_best_point(ranges=data.ranges,start_i=maxGapStart_i, end_i=maxGapEnd_i)
        # print(best_point)
        # self.print_ranges_fancy(data=data)

        """ Both of these statements account for cornering. If we're turning
        either to the left or to the right, check every value in that range.
        If the value of any points in that range are less than a tuned safe
        distance, stop turning and just go straight """
        if (0 < best_point < int(len(data.ranges)/2)):
            for x in range(0, direct_right_index+1):
                if data.ranges[x] < self.SAFE_WALL_DISTANCE:
                    best_point = int(len(data.ranges)/2)
                    break
        elif (int(len(data.ranges)/2) < best_point < len(data.ranges)):
            for x in range(direct_left_index, len(data.ranges)):
                if data.ranges[x] < self.SAFE_WALL_DISTANCE:
                    best_point = int(len(data.ranges)/2)
                    break

        # set the angle value to radians, offset by angle_min
        angle = best_point*data.angle_increment + data.angle_min

        # clamp angle based on anything greater than a wider turn (20 degrees)
        if(angle < self.WIDE_TURN_THRESHOLD*-1): # if <-20, set to -20
            angle = self.WIDE_TURN_THRESHOLD*-1
        elif angle > self.WIDE_TURN_THRESHOLD: # if >20, set to 20
            angle = self.WIDE_TURN_THRESHOLD
        
        # give speeds dependant on our angle 
        # if the best_point is super close, slow down
        if data.ranges[best_point] < self.MAX_ADMISSABLE_DIST:
            self.VELOCITY = self.SHARP_TURN_SPEED
        elif(np.abs(angle) < self.STRAIGHT_AHEAD_THRESHOLD): # < 10 degree speed
            self.VELOCITY = self.STRAIGHT_AHEAD_SPEED # 0-10 degree speed
        elif np.abs(angle) < self.WIDE_TURN_THRESHOLD: # < 20 degree speed
            self.VELOCITY = self.WIDE_TURN_SPEED # 10-20 degree speed
        else: # anything thats not < 20
            self.VELOCITY = self.SHARP_TURN_SPEED # > 20 degree speed

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()