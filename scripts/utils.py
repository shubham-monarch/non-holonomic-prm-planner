import rospy
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseArray, Pose

import sys

def pose_from_point(point):
    pose = Pose()
    pose.position.x = point.x
    pose.position.y = point.y   
    return pose

class RvizPolygon:

    def __init__(self, num_vertices_=4):

        rospy.init_node("rviz_polygon", anonymous=True)
        
        self.clicked_point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_sub)
        self.rviz_polygon_pub = rospy.Publisher("/rviz_polygon", PolygonStamped, queue_size=1)
        self.vertices_pub  = rospy.Publisher("/rviz_polygon_vertices", PoseArray, queue_size=1)
        
        self.num_vertices = num_vertices_
        self.counter = 0 

        

    def clicked_point_sub(self, msg):
    
        print(msg.point.x, msg.point.y, msg.point.z)
        
        if self.counter == 0:
            self.polygon_msg = PolygonStamped()
            self.polygon_msg.header.frame_id = "map"
            self.rviz_polygon_pub.publish(self.polygon_msg) 

            self.vertices = PoseArray()
            self.vertices.header.frame_id  = "map"
            self.vertices_pub.publish(self.vertices)

        self.counter += 1

        pose_ = pose_from_point(msg.point)
        self.vertices.poses.append(pose_)   
        self.vertices_pub.publish(self.vertices)    
        
        self.polygon_msg.polygon.points.append(msg.point)
        

        
        if self.counter == self.num_vertices:
            self.polygon_msg.header.frame_id = "map"
            self.rviz_polygon_pub.publish(self.polygon_msg)
            self.counter = 0
            #self.polygon_msg.polygon.points = []




if __name__ == "__main__":
    
    #print(len(sys.argv))

    if len(sys.argv) < 2:
        RvizPolygon()
    else:
        num_vertices = sys.argv[1]
        RvizPolygon(num_vertices)
    
    rospy.spin()

