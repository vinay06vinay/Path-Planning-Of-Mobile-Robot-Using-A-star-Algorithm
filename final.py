import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import heapq as hq
import time
from collections import defaultdict
def obstacle_space(width,height,canvas,robot_radius,clearance):
    obstacle_matrix = {}
    '''
    Creating a obstacle space with clearance of 5mm in the map and 
    initialising all the coordinates in obstacle space with a cost of -1 and 
    all coordinates in free space with a cost of 'inf' initially. 
    
    The factor of offset that should be multiplied in the line equation is calculated by comparing 
    the y-intercepts of two parallel lines.
    '''
    if(robot_radius > 5 or robot_radius < 0):
        robot_radius = 5
    if(clearance > 5 or clearance < 0):
        clearance = 5
    offset  = robot_radius+clearance
    for x in range(width):
        for y in range(height):
          #Obstacle map and matrix formation with offset consideration
           # #First Rectangle
            r1c = (x+offset>=100 and x-offset<=150) and (y+offset>=0 and y-offset<=100)
            #second Rectangle
            r2c = (x+offset>=100 and x-offset<=150) and (y+offset>=150 and y-offset<=250)
            #Hexagon, representing as sides 's'
            s1c = (x+(1.18*offset)-235>=0)
            s2c = (y+(offset*1.18)+0.576*(x) -223.07 >=0)
            s3c = (0.569*(x)-y-(1.18*offset)-120.90<=0)
            s4c = (x-offset-364.95 <=0)
            s5c = (0.577*(x)+y-(1.18*offset)-373.21<=0)
            s6c = (0.576*(x)-y+(1.18*offset)+26.923 >=0)
            #triangle
            l1c = (x+offset-460>=0)
            l2c = (2*(x)-(2.816*offset)-y-895<=0)
            l3c = (2*(x)-(2.816*offset)+y-1145<=0)
            if(r1c or r2c or (s1c and s2c and s3c and s4c and s5c and s6c) or (l1c and l2c and l3c)):
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,120,255)
            #Creating Boundary with clearance of 5mm
            elif(((y-offset <=0) or (y-250+offset >=0))) :
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,120,255)
            elif(((x-offset <=0) or (x-600+offset >=0))) :
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,120,255)
            else:
                obstacle_matrix[(x,y)] = math.inf
                
                
          #Highlight the original obstacles without clearance with different colour
            # #First Rectangle
            r1 = (x>=100 and x<=150) and (y>=0 and y<=100)
            #second Rectangle
            r2 = (x>=100 and x<=150) and (y>=150 and y<=250)
            #Hexagon, representing as sides 's'
            s1 = (x-235>=0)
            s2 = (y+0.576*x -223.07 >=0)
            s3 = (0.569*x-y-120.90<=0)
            s4 = (x-364.95 <=0)
            s5 = (0.577*x+y-373.21<=0)
            s6 = (0.576*x-y+26.923 >=0)
            #triangle
            l1 = (x-460>=0)
            l2 = (2*x- y-895<=0)
            l3 = (2*x+y-1145<=0)
            if(r1 or r2 or (s1 and s2 and s3 and s4 and s5 and s6) or (l1 and l2 and l3)):
                canvas[y,x] = (0,0,255)
    # plt.imshow(canvas)
    return obstacle_matrix,canvas
def get_start_goal_inputs(obstacle_matrix):
    
    while True:
        print("Enter the x-coordinate of start node")
        start_x = int(input())
        print("Enter the y-coordinate of start node")
        start_y = int(input())
        if((start_x <= 0 or start_x > 599) ):
            print("The X-coordinate of start node is out of the map. Please enter the coordinates again")
        elif((start_y <= 0 or start_y >249)):
            print("The Y-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(obstacle_matrix[(start_x,250-start_y)] == -1):
            print("The entered start node falls in obstacle space")
        else:
            break
    while True:
        print("Initial Orientation of start node of robot(Angle in multiple of 30)")
        start_angle = int(input())
        if(start_angle %3 != 0):
            print("Please enter correct angle ")
        else:
            if(start_angle <30 ):
                start_angle = 360+start_angle
            break
    while True:
        print("Enter the x-coordinate of goal node")
        goal_x = int(input())
        print("Enter the y-coordinate of goal node")
        goal_y = int(input())
        if(goal_x <= 0 or goal_x >599):
            print("The X-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(goal_y <= 0 and goal_y > 249):
            print("The Y-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(obstacle_matrix[(goal_x,250-goal_y)] == -1):
            print("The entered goal node falls in obstacle space")
        else:
            break
    while True:
        print("Goal Orientation of goal node of robot(Angle in multiple of 30)")
        goal_angle = int(input())
        if(goal_angle %3 != 0):
            print("Please enter correct angle ")
        else:
            if(goal_angle <30 ):
                goal_angle = 360+goal_angle
            break
    return (start_x,start_y,start_angle,goal_x,goal_y,goal_angle)
def get_robot_inputs():
    while True:  
        print("Enter the radius of the robot (0-5)")
        robot_radius = int(input())
        if(robot_radius>=0):
            break
    while True:  
        print("Enter the clearance of the robot (0-5)")
        clearance = int(input())
        if(clearance>=0):
            break
    while True:
        print("Enter the step-size of the robot (1-10)")
        step_size = int(input())
        if(step_size<1 or step_size>10):
            print("Please enter valid step_size")
        else:
            break
    return robot_radius,clearance,step_size
'''
5 Move Actions with thetas in {0,30,60,-30,-60}
For Each move action, theta is added to current angle and the coordinates of the new node are found out.
Each movie action also checks whether the node is in obstacle space or not.
'''
def move_0(obstacle_matrix,current_node,step):
    flag = True
    current_node = list(current_node)
    temp_new_node = current_node.copy()
    new_angle = temp_new_node[2] + 0
    if(new_angle < 0):
        new_angle = new_angle+360
    new_x = round((temp_new_node[0] + (step*np.cos(np.radians(new_angle))))*2)/2
    new_y = round((temp_new_node[1] + (step*np.sin(np.radians(new_angle))))*2)/2
    if((round(new_x) >0 and round(new_x) < 600) and (round(new_y) >0 and round(new_y) < 250) and (obstacle_matrix[(int(round(new_x)),int(round(new_y)))] != -1)):
        flag =  True
    else:
        flag = False
        new_node = (new_x,new_y,new_angle)
        return flag,new_node
    new_node = (new_x,new_y,new_angle)
    return flag,new_node
def move_plus_30(obstacle_matrix,current_node,step):
    flag = True
    current_node = list(current_node)
    temp_new_node = current_node.copy()
    new_angle = temp_new_node[2] + 30
    if(new_angle < 0):
        new_angle = new_angle+360
    new_x = round((temp_new_node[0] + (step*np.cos(np.radians(new_angle))))*2)/2
    new_y = round((temp_new_node[1] + (step*np.sin(np.radians(new_angle))))*2)/2
    if((round(new_x) >0 and round(new_x) < 600) and (round(new_y) >0 and round(new_y) < 250) and (obstacle_matrix[(int(round(new_x)),int(round(new_y)))] != -1)):
        flag =  True
    else:
        flag = False
        new_node = (new_x,new_y,new_angle)
    new_node = (new_x,new_y,new_angle)
    return flag,new_node
def move_plus_60(obstacle_matrix,current_node,step):
    flag = True
    current_node = list(current_node)
    temp_new_node = current_node.copy()
    new_angle = temp_new_node[2] + 60
    if(new_angle < 0):
        new_angle = new_angle+360
    new_x = round((temp_new_node[0] + (step*np.cos(np.radians(new_angle))))*2)/2
    new_y = round((temp_new_node[1] + (step*np.sin(np.radians(new_angle))))*2)/2
    if((round(new_x) >0 and round(new_x) < 600) and (round(new_y) >0 and round(new_y) < 250) and (obstacle_matrix[(int(round(new_x)),int(round(new_y)))] != -1)):
        flag =  True
    else:
        flag = False
        new_node = (new_x,new_y,new_angle)
        return flag,new_node
    new_node = (new_x,new_y,new_angle)
    return flag,new_node
def move_minus_30(obstacle_matrix,current_node,step):
    flag = True
    current_node = list(current_node)
    temp_new_node = current_node.copy()
    new_angle = temp_new_node[2] -30
    if(new_angle < 0):
        new_angle = new_angle+360
    new_x = round((temp_new_node[0] + (step*np.cos(np.radians(new_angle))))*2)/2
    new_y = round((temp_new_node[1] + (step*np.sin(np.radians(new_angle))))*2)/2
    if((round(new_x) >0 and round(new_x) < 600) and (round(new_y) >0 and round(new_y) < 250) and (obstacle_matrix[(int(round(new_x)),int(round(new_y)))] != -1)):
        flag =  True
    else:
        flag = False
        new_node = (new_x,new_y,new_angle)
        return flag,new_node
    new_node = (new_x,new_y,new_angle)
    return flag,new_node
def move_minus_60(obstacle_matrix,current_node,step):
    flag = True
    current_node = list(current_node)
    temp_new_node = current_node.copy()
    new_angle = temp_new_node[2] - 60
    if(new_angle < 0):
        new_angle = new_angle+360
    new_x = round((temp_new_node[0] + (step*np.cos(np.radians(new_angle))))*2)/2
    new_y = round((temp_new_node[1] + (step*np.sin(np.radians(new_angle))))*2)/2
    if((round(new_x) >0 and round(new_x) < 600) and (round(new_y) >0 and round(new_y) < 250) and (obstacle_matrix[(int(round(new_x)),int(round(new_y)))] != -1)):
        flag =  True
    else:
        flag = False
        new_node = (new_x,new_y,new_angle)
        return flag,new_node
    new_node = (new_x,new_y,new_angle)
    return flag,new_node
'''
This Function Calculates the cost to goal between current and goal nodes using Euclidean Distance Formula
'''
def cost_2_goal(start_x,start_y,goal_x,goal_y):
    point_1 = np.array((start_x,start_y))
    point_2 = np.array((goal_x,goal_y))
    cost = np.linalg.norm(point_1-point_2)
    return cost
'''
To check whether the node is already visited. If so, returns true and updates the total cost if its less
'''
def check_duplicate(new_node,visited_matrix):
    if(new_node[0] > 599 or new_node[1] > 249):
        return True
    angle = new_node[2]
    if(angle>=360):
        angle%=360
    if(visited_matrix[int(new_node[0]*2)][int(new_node[1]*2)][int(angle/30)] == 0):
        visited_matrix[int(new_node[0]*2)][int(new_node[1]*2)][int(angle/30)] = 1
        return False,visited_matrix
    else:
        
        return True,visited_matrix
       
def astar(obstacle_matrix,start_x,start_y,start_theta,goal_x,goal_y,goal_theta,step):
    visited_matrix = np.zeros((1200,500,12))
    closed_list = {} #List of nodes explored
    open_list = []
    explored_dict = defaultdict(list) #Used for node explore visualisation where each key is a parent node with all child nodes appended
    child_parent_dict = {} #Used for backtracking and to get the optimal path
    parent_node = (start_x,start_y,start_theta)
    initial_cost_2_come = 0
    initial_cost_2_goal = cost_2_goal(start_x,start_y,goal_x,goal_y)
    total_cost = initial_cost_2_come+initial_cost_2_goal
    hq.heapify(open_list)
    hq.heappush(open_list,[total_cost,initial_cost_2_come,parent_node,(start_x,start_y,start_theta)])
    count = 0
    goal_reached = False
    explored_dict[parent_node].append((parent_node))
    while(len(open_list)>0):
        cn=hq.heappop(open_list)
        current_node = cn[3]
        parent_node = cn[2]
        current_cost2c = cn[1]
        current_total_cost = cn[0]
        closed_list[current_node] = cn[2]  
        explored_dict[(int(parent_node[0]),int(parent_node[1]),int(parent_node[2]))].append((int(current_node[0]),int(current_node[1]),int(current_node[2]))) 
        child_parent_dict[((current_node[0]),(current_node[1]))] = ((parent_node[0]),(parent_node[1])) 
        goal_check_distance = np.sqrt(((goal_y-current_node[1])**2)+((goal_x-current_node[0])**2))
        if(goal_check_distance<1.5):
            goal_reached = True
            print("Goal Reached, Starting Back Track")
            break
        action_set = {move_0:"move_zero",move_plus_30:"move_plus_thirty",move_plus_60:"move_plus_sixty",move_minus_30:"move_minus_thirty",move_minus_60:"move_minus_60"}
        
        for action,action_value in action_set.items():
            flag,new_node = action(obstacle_matrix,current_node,step)
            if(flag):
                if(new_node not in closed_list):
                    check_duplicate_flag,visited_matrix = check_duplicate(new_node,visited_matrix)
                    if(check_duplicate_flag):
                        for i in open_list:
                            if (new_node == i[3]):
                                cost = current_cost2c+step+cost_2_goal(new_node[0],new_node[1],goal_x,goal_y)
                                if(cost < i[0]):
                                    i[0] = cost
                                    i[1] = step+current_cost2c
                                    i[2] = cn[2]
                                    hq.heapify(open_list)
                                    child_parent_dict[(int(new_node[0]),int(new_node[1]))] = (int(current_node[0]),int(current_node[1])) 
                                break
                    else:
                        cost = current_cost2c+step+cost_2_goal(new_node[0],new_node[1],goal_x,goal_y)
                        c2c = current_cost2c+step
                        node_to_push = [cost,c2c,current_node,new_node]
                        hq.heappush(open_list,node_to_push)
                        # explored_dict[(int(current_node[0]),int(current_node[1]),int(current_node[2]))].append((int(new_node[0]),int(new_node[1]),int(new_node[2])))
                        hq.heapify(open_list)
                   
        count += 1 
    return(child_parent_dict,goal_reached,closed_list,explored_dict,closed_list)
'''
Backtracking using the child parent dictionary relationship got from the exploration
'''
def back_track(child_parent_dict,start_x,start_y,start_theta,goal_x,goal_y,goal_theta):
    start = (start_x,start_y)
    optimal_list = []
    optimal_list.append((goal_x,goal_y))
    if((float(goal_x),float(goal_y)) in child_parent_dict.keys()):
        current = (float(goal_x),float(goal_y))
    else:
        last_key = list(child_parent_dict)[-1]
        current = last_key
    while(start!=current):
        if (current in child_parent_dict.keys()):
            optimal_list.append(child_parent_dict[current])
            current = child_parent_dict[current]
        else:
            break
    return optimal_list[::-1]
'''
Visualizing and saving the video using OpenCV
'''
def visualise(canvas,optimal_list,explored_dict,closed_list,child_parent_dict):
    fourcc = cv2.VideoWriter_fourcc(*'XVID')    
    out = cv2.VideoWriter('A-Star.avi',fourcc,250,(canvas.shape[1],canvas.shape[0]))
    # for key,value in child_parent_dict.items():
    #     cv2.arrowedLine(canvas, (int(value[0]),int(value[1])), (int(key[0]),int(key[1])), (255,255,255), 1,tipLength=0.5) 
    #     out.write(canvas)
    for key,values in explored_dict.items():
        for i in values:
            pt1 = key[:2]
            pt2 = i[:2]
            cv2.arrowedLine(canvas, pt1, pt2, (255,255,255), 1,tipLength=0.5) 
        # cv2.imshow("Exploration",canvas)
        out.write(canvas)
        cv2.waitKey(1)
    # for key,value in closed_list.items():
    #     parent_pt = (int(value[0]),int(value[1]))
    #     child_pt = (int(key[0]),int(key[1]))
    #     cv2.arrowedLine(canvas, parent_pt, child_pt, (255,255,255), 1) 
    #     out.write(canvas)
    for i in range(len(optimal_list)-1):
        pt1 = (int(optimal_list[i][0]),int(optimal_list[i][1]))
        pt2 = (int(optimal_list[i+1][0]),int(optimal_list[i+1][1]))
        cv2.line(canvas, pt1, pt2, (0,255,0), 1,cv2.LINE_AA)
        # cv2.imshow("Optimal",canvas)
        # cv2.waitKey(1)
        out.write(canvas)
    for i in range(500):
        out.write(canvas)
    out.release()
    cv2.destroyAllWindows()
    plt.imshow(canvas)
def main():
    canvas = np.ones((250,600,3),dtype='uint8')
    robot_radius,clearance,step_size = get_robot_inputs()
    obstacle_matrix,canvas = obstacle_space(600,250,canvas,robot_radius,clearance) 
    start_x,start_y,start_theta,goal_x,goal_y,goal_theta = get_start_goal_inputs(obstacle_matrix)
    
    # #Changing the y-coordinates in relation to image coordinates
    start_y = 250-start_y
    goal_y = 250-goal_y
    print(f"The start node selected : {(start_x,start_y,start_theta)}. The goal node selected : {(goal_x,goal_y,goal_theta)}")
    start_time = time.time()
    child_parent_dict,goal_reached,closed_list,explored_dict,closed_list = astar(obstacle_matrix,start_x,start_y,start_theta,goal_x,goal_y,goal_theta,step_size)
    end_time = time.time()  
    print(f"Time Taken By the Algorithm : {end_time - start_time} seconds")
    if(goal_reached == False):
        print("No Goal Found")
        return
    print("The length of visited or explored list",len(closed_list))
    optimal_list = back_track(child_parent_dict,start_x,start_y,start_theta,goal_x,goal_y,goal_theta)
    print("The Optimal Path Node:",optimal_list)
    visualise(canvas,optimal_list,explored_dict,closed_list,child_parent_dict)
if __name__ == '__main__':
    
    main()
    
    