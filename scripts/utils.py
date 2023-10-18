import rospy
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseArray, Pose
from shapely.geometry import Polygon, Point
import sys
import random

'''
generates a polygon in rviz by clicking on the map
'''

def pose_from_point(point):
    pose = Pose()
    pose.position.x = point.x
    pose.position.y = point.y   
    
    return pose

def getPoseArrayFromPoses(poses: list):
    
    sampled_points_arr = PoseArray()
    sampled_points_arr.header.frame_id = "map"
    sampled_points_arr.header.stamp = rospy.Time.now()
    sampled_points_arr.poses = [pose_from_point(point) for point in poses]
    return sampled_points_arr


def sample_points(num_points: int, polygon_ros: PolygonStamped):
    
    print("sample_points called!")
    

    print("Printing ros polygon vertices: \n")

    for point in polygon_ros.polygon.points:
        print("({},{})".format(point.x, point.y))

    sampled_points = []
   
    polygon_vertices = [] 

    cnt = 0 
    for point in polygon_ros.polygon.points:
        cnt += 1
        polygon_vertices.append((point.x, point.y))
        print ("cnt: ", cnt)

    polygon = Polygon(polygon_vertices)

    (mn_x, mn_y, mx_x, mx_y) = polygon.bounds
 
    while len(sampled_points) < num_points:
        x = random.uniform(mn_x, mx_x)
        y = random.uniform(mn_y, mx_y)
        point = Point(x, y)
        if point.within(polygon):
            sampled_points.append(point)
    
    return sampled_points

class RvizPolygon:

    def __init__(self, num_vertices_=4):
        self.clicked_point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb)
        
        self.rviz_polygon_pub = rospy.Publisher("/rviz_polygon", PolygonStamped, queue_size=1, latch=True)
        self.rviz_vertices_pub  = rospy.Publisher("/rviz_polygon_vertices", PoseArray, queue_size=1)
        self.rviz_sampled_points_pub = rospy.Publisher("/rviz_sampled_points", PoseArray, queue_size=1, latch= True)
        #self.sampled_points_pub = rospy.Publisher("/rviz_sampled_points", PoseArray, queue_size=1)
        
        self.num_vertices = num_vertices_
        self.num_sample_points = 500
        self.counter = 0 
        self.polygon_msg = PolygonStamped()
        self.polygon_vertices = PoseArray()
        self.sampled_points = PoseArray()

        
    def clicked_point_cb(self, msg):
    
        print(msg.point.x, msg.point.y, msg.point.z)
        print("self.counter: ", self.counter)
        
        if self.counter == 0:
            self.polygon_msg = PolygonStamped()
            self.polygon_msg.header.frame_id = "map"
            self.rviz_polygon_pub.publish(self.polygon_msg) 

            self.polygon_vertices = PoseArray()
            self.polygon_vertices.header.frame_id  = "map"
            self.rviz_vertices_pub.publish(self.polygon_vertices)

            self.sampled_points = PoseArray()
            self.sampled_points.header.frame_id = "map"
            self.rviz_sampled_points_pub.publish(PoseArray())   

        self.counter += 1

        pose_ = pose_from_point(msg.point)
        self.polygon_vertices.poses.append(pose_)   
        self.rviz_vertices_pub.publish(self.polygon_vertices)    
        
        self.polygon_msg.polygon.points.append(msg.point)
        

        
        if self.counter == self.num_vertices:
            
            #publish polygon 
            self.polygon_msg.header.frame_id = "map"
            self.rviz_polygon_pub.publish(self.polygon_msg)
            
            #reset counter
            self.counter = 0
            #self.polygon_msg.polygon.points = []
            
            #publish sampled points
            sampled_points_ = sample_points(self.num_sample_points, self.polygon_msg)
            self.sampled_points = getPoseArrayFromPoses(sampled_points_) 
            self.rviz_sampled_points_pub.publish(self.sampled_points)
            



if __name__ == "__main__":
    
    #print(len(sys.argv))
    rospy.init_node("rviz_polygon", anonymous=False)
        
    if len(sys.argv) < 2:
        RvizPolygon()
    else:
        num_vertices = sys.argv[1]
        RvizPolygon(num_vertices)
    
    rospy.spin()

