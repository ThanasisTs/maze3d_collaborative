from maze3D_new.config import *
from maze3D_new.assets import *
import math
import numpy as np
from scipy.spatial import distance
import time

class GameBoard:
    def __init__(self,layout):
        self.velocity = [0, 0]
        self.walls = []
        self.layout = layout
        for row in range(len(layout)):
            self.walls.append([])
            for col in range(len(layout[0])):
                self.walls[row].append(None)
                if layout[row][col] != 0:
                    if layout[row][col] == 2:
                        self.hole = Hole(32*col - 150, 32*row - 150, self)
                    elif layout[row][col] == 3:
                        self.ball = Ball((32*col) - 150, (32*row) -150, self)
                    else:
                        self.walls[row][col] = Wall((32*col) - 160,(32*row) - 160, layout[row][col], self)


        self.rot_x = 0
        self.rot_y = 0
        self.max_x_rotation = 0.5
        self.max_y_rotation = 0.5
        self.count_slide = 0
        self.slide = False
        self.last_command_x, self.last_command_y = 0, 0

        self.keyMap = {1:(1,0),
                        2:(-1,0),
                        4:(0,1),5:(1,1),6:(-1,1),7:(0,1),
                        8:(0,-1),9:(1,-1),10:(-1,-1),11:(0,-1),13:(1,0),14:(-1,0)}
    
    def getBallCoords(self):
        return (self.ball.x, self.ball.y)

    def collideSquare(self, x, y):
        # if the ball hits a square obstacle, it will return True
        # and the collideTriangle will not be called
        xGrid = math.floor(x / 32 + 5)
        yGrid = math.floor(y / 32 + 5)
        biggest = max(xGrid, yGrid)
        smallest = min(xGrid, yGrid)
        if biggest > 13 or smallest < 0:
            return True
        if self.walls[yGrid][xGrid] != None:
            if self.layout[yGrid][xGrid] == 1:
                return True
        return False

    def collideTriangle(self, checkX, checkY, x, y, velx, vely, thetaX, thetaY):
        # find the grid that the ball tends to enter
        # grid_directionX stores the coordinates of the ball in the x axis
        # grid_directionY stores the coordinates of the ball in the y axis
        
        count = 0
        grid_directionX = [math.floor(checkX/32 + 5), math.floor(y/32 + 5)]
        grid_directionY = [math.floor(x/32 + 5), math.floor(checkY/32 + 5)]

        check_collision = [grid_directionX, grid_directionY]

        # check that the ball moves in the obstacle-free space
        for direction in check_collision:
            biggest = max(direction[0],direction[1])
            smallest = min(direction[0],direction[1])

            # if the grid has an object
            if self.layout[direction[1]][direction[0]] == 0 and not self.slide:
                return velx, vely, False

        # if code reaches this point, we are at a grid of triangle obstacle

        # change reference point to be down left pixel of the grid
        xObs, yObs = 0, 0
        xBall, yBall = x-32*direction[0] + 160, y-32*direction[1] + 160

        # get the point of the ball that will hit the triangle obstacle
        xCol4, yCol4 = x + 8*np.cos(225*np.pi/180), y + 8*np.sin(225*np.pi/180)
        xGridCol4, yGridCol4 = math.floor(xCol4/32+5), math.floor(yCol4/32+5)
        xCol5, yCol5 = x + 8*np.cos(45*np.pi/180), y + 8*np.sin(45*np.pi/180)
        xGridCol5, yGridCol5 = math.floor(xCol5/32+5), math.floor(yCol5/32+5)

        # left triangle object
        if self.layout[yGridCol4][xGridCol4] == 4:
            
            # collision angle
            theta = 225*np.pi/180
            xCol, yCol = xBall + 8*np.cos(theta), yBall + 8*np.sin(theta)
            thetaCol = np.arctan((yCol-yObs)/(xCol-32-xObs))*180/np.pi
            
            if thetaCol > -10:
                thetaCol = 135.5
            else:
                thetaCol += 180

            # if thetaCol is less than 133 degrees, reset the slide counter and the slide flag
            # and return the commanded velocities
            if thetaCol < 133:
                self.count_slide = 0
                self.slide = False
                self.step = 0
                return velx, vely, False
            # if thetaCol is between 133 and 135 degrees, decrease the slide counter and 
            # return the commanded velocities
            elif 133 <= thetaCol <= 135:
                return velx, vely, False
            # if thetaCol is greater than 135 degrees, then the ball hit the triangle
            elif thetaCol > 135:
                self.count_slide += 1
                # if collision angle is greater than 135 degrees 10 consecutive times, 
                # then we assume that the ball touches the leaning surface. Otherwise, the ball
                # will bounce 
                if self.count_slide == 3:
                    self.slide = True
                    print('I touch the surface')
                elif not self.slide:
                    print('gonna bounch')
                    return 0, 0, True
                if self.slide:
                    print(thetaX, thetaY)
                    if thetaY < 0 and thetaX > 0:
                        if abs(thetaY) > abs(thetaX):
                            if self.collideSquare(x+8, y):
                                if thetaY <= 0:
                                    return 0, 0, True
                                else:
                                    return 0, vely, False
                            return 0.1, -0.1, False
                        else:
                            if self.collideSquare(x, y+8):
                                if thetaX >= 0:
                                    return 0, 0, True
                                else:
                                    return velx, 0, False
                            return -0.1, 0.1, False
                    else:
                        if thetaX < 0 and thetaY > 0:
                            return velx, vely, False
                        else:
                            if thetaX >= 0:
                                if self.collideSquare(x+8, y):
                                    if thetaY <= 0:
                                        return 0, 0, True
                                    else:
                                        return velx, 0, False
                                return -0.1*np.cos(thetaCol*np.pi/180), vely, False
                            else:
                                if self.collideSquare(x, y+8):
                                    if thetaX >= 0:
                                        return 0, 0, True
                                    else:
                                        return velx, 0, False
                                return velx, -0.1*np.sin(thetaCol*np.pi/180), False
        
        # right triangle
        elif self.layout[yGridCol5][xGridCol5] == 5:
            
            # collision angle
            theta = 45*np.pi/180
            xCol, yCol = xBall + 8*np.cos(theta), yBall + 8*np.sin(theta)
            thetaCol = np.arctan((yCol-yObs)/(xCol-32-xObs))*180/np.pi
            
            if thetaCol < -50 or thetaCol > -25:
                thetaCol = 134.5
            else:
                thetaCol += 180

            # if thetaCol is less than 133 degrees, reset the slide counter and the slide flag
            # and return the commanded velocities
            if thetaCol > 137:
                self.count_slide = 0
                self.slide = False
                return velx, vely, False
            # if thetaCol is between 133 and 135 degrees, decrease the slide counter and 
            # return the commanded velocities
            elif 135 <= thetaCol <= 137:
                return velx, vely, False
            # if thetaCol is greater than 135 degrees, then the ball hit the triangle
            elif thetaCol < 135:
                self.count_slide += 1
                # if collision angle is greater than 135 degrees 10 consecutive times, 
                # then we assume that the ball touches the leaning surface. Otherwise, the ball
                # will bounce 
                if self.count_slide == 3:
                    self.slide = True
                    print('I touch the surface')
                elif not self.slide:
                    print('gonna bounch')
                    return 0, 0, True
                if self.slide:

                    if thetaY > 0 and thetaX < 0:
                        if abs(thetaY) > abs(thetaX):
                            if self.collideSquare(x, y-8):
                                if thetaX <= 0:
                                    return velx, 0, True
                                else:
                                    return 0, 0, False
                            return 0.1*np.cos(thetaCol*np.pi/180), 0.1*np.sin(thetaCol*np.pi/180), False
                        else:
                            if self.collideSquare(x-8, y):
                                if thetaY <= 0:
                                    return 0, vely, True
                                else:
                                    return 0, 0, False
                            return -0.1*np.cos(thetaCol*np.pi/180), -0.1*np.sin(thetaCol*np.pi/180), False
                    else:
                        if thetaX >= 0 and thetaY <= 0:
                            return velx, vely, False
                        else:
                            if thetaX < 0:
                                if self.collideSquare(x, y-8):
                                    if thetaX <= 0:
                                        return velx, 0, True
                                    else:
                                        return 0, 0, False                                
                                return 0.1*np.cos(thetaCol*np.pi/180), vely, False
                            else:
                                if self.collideSquare(x-8, y):
                                    if thetaY <= 0:
                                        return 0, vely, True
                                    else:
                                        return 0, 0, False
                                return velx, 0.1*np.sin(thetaCol*np.pi/180), False
        count += 1
        return velx, vely, False

    
    def update(self):
        #compute rotation matrix
        rot_x_m = pyrr.Matrix44.from_x_rotation(self.rot_x)
        rot_y_m = pyrr.Matrix44.from_y_rotation(self.rot_y)
        self.rotationMatrix = pyrr.matrix44.multiply(rot_x_m,rot_y_m)

        self.ball.update()
        self.hole.update()

        for row in self.walls:
            for wall in row:
                if wall != None:
                    wall.update()

    def handleKeys_fotis(self, angleIncrement):
        if angleIncrement[0] == 2:
            angleIncrement[0] = -1
        if angleIncrement[1] == 2:
            angleIncrement[1] = -1
        self.velocity[0] = 0.01 * angleIncrement[0]
        self.rot_x += self.velocity[0]
        if self.rot_x >= self.max_x_rotation:
            self.rot_x = self.max_x_rotation
            self.velocity[0] = 0
        elif self.rot_x <= -self.max_x_rotation:
            self.rot_x = -self.max_x_rotation
            self.velocity[0] = 0

        self.velocity[1] = 0.01 * angleIncrement[1]
        self.rot_y += self.velocity[1]
        if self.rot_y >= self.max_y_rotation:
            self.rot_y = self.max_y_rotation
            self.velocity[1] = 0
        elif self.rot_y <= -self.max_y_rotation:
            self.rot_y = -self.max_y_rotation
            self.velocity[1] = 0

    def handleKeys(self,key):
        if key in self.keyMap:
            angleIncrement = self.keyMap[key]
            self.rot_x += 0.01*angleIncrement[0]
            if self.rot_x >= self.max_x_rotation:
                self.rot_x = self.max_x_rotation
            elif self.rot_x <= -self.max_x_rotation:
                self.rot_x = -self.max_x_rotation
            self.rot_y += 0.01*angleIncrement[1]
            if self.rot_y >= self.max_y_rotation:
                self.rot_y = self.max_y_rotation
            elif self.rot_y <= -self.max_y_rotation:
                self.rot_y = -self.max_y_rotation
    
    def draw(self, mode=False, idx=0):
        glUniformMatrix4fv(MODEL_LOC,1,GL_FALSE,self.rotationMatrix)
        glBindVertexArray(BOARD_MODEL.getVAO())
        glBindTexture(GL_TEXTURE_2D,BOARD.getTexture())
        glDrawArrays(GL_TRIANGLES,0,BOARD_MODEL.getVertexCount())

        self.ball.draw()
        self.hole.draw()

        for row in self.walls:
            for wall in row:
                if wall != None:
                    wall.draw()
        if mode:
            translation = pyrr.matrix44.create_from_translation(pyrr.Vector3([0, 400, 0]))
            glUniformMatrix4fv(MODEL_LOC,1,GL_FALSE,pyrr.matrix44.multiply(translation, pyrr.matrix44.create_identity()))
            glBindVertexArray(TEXT_MODEL.getVAO())
            glBindTexture(GL_TEXTURE_2D,TEXT[idx].getTexture())
            glDrawArrays(GL_TRIANGLES,0,TEXT_MODEL.getVertexCount())

class Wall:
    def __init__(self,x,y,type,parent):
        self.parent = parent
        self.x = x
        self.y = y
        self.z = 0
        self.type = type-1

    def update(self):
        #first translate to position on board, then rotate with the board
        translation = pyrr.matrix44.create_from_translation(pyrr.Vector3([self.x,self.y,self.z]))
        self.model = pyrr.matrix44.multiply(translation,self.parent.rotationMatrix)
    
    def draw(self):
        glUniformMatrix4fv(MODEL_LOC,1,GL_FALSE,self.model)
        glBindVertexArray(WALL_MODELS[self.type].getVAO())
        glBindTexture(GL_TEXTURE_2D,WALL.getTexture())
        glDrawArrays(GL_TRIANGLES,0,WALL_MODELS[self.type].getVertexCount())

class Ball:
    def __init__(self,x,y,parent):
        self.parent = parent
        self.x = x
        self.y = y
        self.z = 0
        self.velocity = [0,0]
    
    def update(self):
        #first translate to position on board, then rotate with the board
        translation = pyrr.matrix44.create_from_translation(pyrr.Vector3([self.x,self.y,self.z]))
        self.model = pyrr.matrix44.multiply(translation,self.parent.rotationMatrix)
        acceleration = [-0.1*self.parent.rot_y,0.1*self.parent.rot_x]
        self.velocity[0] += acceleration[0]
        self.velocity[1] += acceleration[1]

        cmd_vel_x = self.velocity[0]
        cmd_vel_y = self.velocity[1]

        check_collision_X = self.x + self.velocity[0] + 8*np.sign(self.velocity[0])
        check_collision_Y = self.y + self.velocity[1] + 8*np.sign(self.velocity[1])

        nextX = self.x + self.velocity[0]
        nextY = self.y + self.velocity[1]

        # check x direction
        checkXCol = self.parent.collideSquare(check_collision_X, self.y)
        checkYCol = self.parent.collideSquare(self.x, check_collision_Y)

        if checkXCol:
            self.velocity[0] *= -0.25
        # check y direction
        if checkYCol:
            self.velocity[1] *= -0.25


        if not checkXCol and not checkYCol:
            velx, vely, collision = self.parent.collideTriangle(check_collision_X, check_collision_Y, nextX, nextY, cmd_vel_x, cmd_vel_y, self.parent.rot_y, self.parent.rot_x)
            if collision:
                self.velocity[0] *= -0.25
                self.velocity[1] *= -0.25
            else:
                self.velocity = [velx, vely]

        self.x += self.velocity[0]
        self.y += self.velocity[1]
    
    def draw(self):
        glUniformMatrix4fv(MODEL_LOC,1,GL_FALSE,self.model)
        glBindVertexArray(BALL_MODEL.getVAO())
        glBindTexture(GL_TEXTURE_2D,BALL.getTexture())
        glDrawArrays(GL_TRIANGLES,0,BALL_MODEL.getVertexCount())

class Hole:
    def __init__(self, x, y, parent):
        self.parent = parent
        self.x = x
        self.y = y
        self.z = 0

    def update(self):
        # first translate to position on board, then rotate with the board
        translation = pyrr.matrix44.create_from_translation(pyrr.Vector3([self.x, self.y, self.z]))
        self.model = pyrr.matrix44.multiply(translation, self.parent.rotationMatrix)

    def draw(self):
        glUniformMatrix4fv(MODEL_LOC, 1, GL_FALSE, self.model)
        glBindVertexArray(HOLE_MODEL.getVAO())
        glBindTexture(GL_TEXTURE_2D, HOLE.getTexture())
        glDrawArrays(GL_TRIANGLES, 0, HOLE_MODEL.getVertexCount())
