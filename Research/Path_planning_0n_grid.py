## Import OpenCV
import cv2
## Import heapq
import heapq ##priority queue
##heapq description###
#Also known as priority queue	
#The heapq module maintains the heap invariant, which is not the same thing as maintaining the actual list object in sorted order.
#Quoting from the heapq documentation:
#Heaps are binary trees for which every parent node has a value less than or equal to any of its children.
#This implementation uses arrays for which heap[k] <= heap[2*k+1] and heap[k] <= heap[2*k+2] for all k, counting elements from zero.
#For the sake of comparison, non-existing elements are considered to be infinite.
#The interesting property of a heap is that its smallest element is always the root, heap[0]
##################################
import numpy as np
############################################
############################################
## Read the image
grid_map= [ [ 0 for i in range(10) ] for j in range(10) ]

grid_start=0
grid_end=0

def solve(start,finish,img): #no heuristics used
    """Find the shortest path from START to FINISH."""
    
    heap=[]
    link = {} # parent node link
    g = {} # shortest path to a current node
    g[start] = 0 #initial distance to node start is 0
    
    link[start] = None #parent of start node is none
    
    
    heapq.heappush(heap, (0, start))
    
    while True:
        
        f, current = heapq.heappop(heap) ##taking current node from heap
        #print current
        if current == finish:
            name='Shortest Path, image#'
            i=int(100*np.random.rand())
            name=name+str(i)
            route=build_path(start, finish, link)
            ####Drawing path , just for pictorial representation######
            for i in range(1,len(route)):
                cv2.line(img,(route[i-1].y*40+20,route[i-1].x*40+20),(route[i].y*40+20,route[i].x*40+20),(232,162,0), 3)
            cv2.imshow(name,img)
            ############################
            return g[current], route[1:len(route)]
            
        
        moves = current.get_moves()
        cost = g[current]
        for mv in moves:
            #print mv.x,mv.y
            if grid_map[mv.x][mv.y]==1: #bypass obstacles
                continue
                #mv is the neighbour of current cell, in all maximum 4 neighbours will be there
            if  (mv not in g or g[mv] > cost + 1): #check if mv is already visited or if its cost is higher than available cost then update it
                g[mv] = cost + 1
                
                link[mv] = current #storing current node as parent to mv 
                heapq.heappush(heap, (g[mv], mv)) ##adding updated cost and visited node to heap

    

    
def build_path(start, finish, parent):
    
    #create path from start to finish

    x = finish ##back tracking the path from goal to start
    xs = [x]
    while x != start: #going back
        x = parent[x]
        xs.append(x)
    xs.reverse()
 
    return xs


class GridPoint(object):
    """Represent a position on a grid."""
    def __init__(self, x, y): #self referencing x and  y coordinates
        self.x = x
        self.y = y

    def __hash__(self): #returning hash value of the GridPoint object
        return hash((self.x, self.y))

    def __repr__(self):                         #returns values stored in current object, values are x and y coordinates
        return "(%d,%d)" % (self.y+1, self.x+1)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def get_moves(self): ##taking current node coordinates to find neighbours of it
        
        
        if self.x>=0 and self.x<=len(grid_map)-1 and self.y>=0 and self.y<=len(grid_map)-1:
            if self.x + 1<len(grid_map):
                yield GridPoint(self.x + 1, self.y)
            if self.y + 1<len(grid_map):  
                yield GridPoint(self.x, self.y + 1)
            if self.x - 1>=-1:
                yield GridPoint(self.x - 1, self.y)
            if self.y - 1>=-1:
                yield GridPoint(self.x, self.y - 1)
                


def play(img):
    '''
    img-- a single test image as input argument
    route_length  -- returns the single integer specifying the route length
    '''
    global grid_map
    xs=0 #start coordinates, depicting  horizontal rows in grid_map, vertical column for image
    ys=0 #start coordinates, depicting  vertical column in grid_map, horizontal row for image
    xe=0 #end coordinates
    ye=0
    

    #creating 10x10 matrix space map with black as obstable and other colors as paths.
    for x in range(0, 10):
        X=x*40+20
        for y in range(0,10):
            Y=y*40+20
            #print "Pixels ",Y,X," pixel value  b= ",img[Y,X,0]," g= ",img[Y,X,1]," r= ",img[Y,X,2]
            #cv2.circle(img,(Y,X), 5, (0,0,255), -1)
            if img[Y,X,0]==232 and img[Y,X,1]==162 and img[Y,X,2]==0: #start point blue, bgr value(232,162,0)
                xs=x+1
                ys=y+1
                #cv2.circle(img,(X,Y), 5, (0,255,0), -1)
            elif img[Y,X,0]==0 and img[Y,X,1]==242 and img[Y,X,2]==255: #end/goal point yellow ,bgr value(0,242,255)
                xe=x+1
                ye=y+1
                #cv2.circle(img,(X,Y), 5, (0,0,255), -1)
            elif img[Y,X,0]==0 and img[Y,X,1]==0 and img[Y,X,2]==0: #obstacle black ,bgr value(0,0,0)
                grid_map[y][x]=1
                #cv2.circle(img,(X,Y), 5, (0,0,255), -1) 
            continue
    
    grid_start = GridPoint(ys-1,xs-1) ##reversing coordinates so that it can be compatible with coordinate system of matrix
    grid_end = GridPoint(ye-1,xe-1)
    route_length, route_path=solve(grid_start,grid_end,img)
    grid_map=np.zeros((10,10)) #resetting grid map to 0s
    return route_length, route_path #return the values calculated


if __name__ == "__main__":
    #code for checking output for single image
    img = cv2.imread('test_images/test_image1.png')
    route_length, route_path = play(img)
    print "route length = ", route_length
    print "route_path   = ", route_path
    #code for checking output for all images
    route_length_list = []
    route_path_list   = []    
    for file_number in range(1,6):
        file_name = "test_images/test_image"+str(file_number)+".png"
        pic = cv2.imread(file_name)
        route_length, route_path = play(pic)
        #gc.collect()
        route_length_list.append(route_length)
        route_path_list.append(route_path)
    print route_length_list
    print route_path_list
############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
############################################
