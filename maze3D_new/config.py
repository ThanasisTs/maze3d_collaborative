import pygame as pg
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram,compileShader
from OpenGL.GLU import *
import numpy as np
import pyrr
import pywavefront as pwf
from maze3D_new.assets import *



def text_objects(text, color):
    textSurface = font.render(text, True, color)
    return textSurface, textSurface.get_rect()


def message_to_screen(msg, color, param1, size):
    textSurf, textRect = text_objects(msg, color)
    textRect.center = (display_width / 2), (display_height / 2)
    gameDisplay.blit(textSurf, textRect)


def pause():
    pause = True
    while pause:
        for event in pg.event.get():
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_SPACE:
                    pause = False

        # message_to_screen("Paused", BLACK, -100, size="large")
        # gameDisplay.update()
        # clock.tick(5)

pg.font.init()
# font = pg.font.SysFont("Grobold", 20)  # Assign it to a variable font


pg.init()

display = (800, 800)
screen = pg.display.set_mode(display,pg.OPENGL|pg.DOUBLEBUF)
clock = pg.time.Clock()

font = pg.font.Font('freesansbold.ttf', 32)

texts, textsRect = [], []
for i in range(5):
	texts.append(font.render("Game starts in {}...".format(i+1), True, (255, 255, 255)))
	textsRect.append(texts[-1].get_rect())
	textsRect[-1].center = (100, 100)


glClearColor(0.1, 0.2, 0.3, 1)

with open("maze3D_new/shaders/vertex.txt",'r') as f:
    vertex_src = f.readlines()
with open("maze3D_new/shaders/fragment.txt",'r') as f:
    fragment_src = f.readlines()
shader = compileProgram(compileShader(vertex_src,GL_VERTEX_SHADER),
                        compileShader(fragment_src,GL_FRAGMENT_SHADER))
glUseProgram(shader)

#get a handle to the rotation matrix from the shader
MODEL_LOC = glGetUniformLocation(shader,"model")
VIEW_LOC = glGetUniformLocation(shader,"view")
PROJ_LOC = glGetUniformLocation(shader,"projection")
LIGHT_LOC = glGetUniformLocation(shader,"lightPos")

glEnable(GL_DEPTH_TEST)
glEnable(GL_BLEND)
glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
glEnable(GL_CULL_FACE)

########################MODELS######################################
BOARD_MODEL = ObjModel("maze3D_new/models/board.obj")
BALL_MODEL = ObjModel("maze3D_new/models/ball.obj")
WALL_MODELS = [ObjModel("maze3D_new/models/wall.obj"), ObjModel("maze3D_new/models/wall_half_1_big.obj"),
				ObjModel("maze3D_new/models/wall_half_2_big.obj"), ObjModel("maze3D_new/models/wall_half_corner_1_big.obj"),
				ObjModel("maze3D_new/models/wall_half_corner_2_big.obj")]
HOLE_MODEL = ObjModel("maze3D_new/models/hole.obj")
TEXT_MODEL = ObjModel("maze3D_new/models/text.obj")
########################TEXTURES####################################
BOARD = Texture("maze3D_new/textures/board_white.png")
WALL = Texture("maze3D_new/textures/wall_simple.jpg")
BALL = Texture("maze3D_new/textures/glass.png")
HOLE = Texture("maze3D_new/textures/green.png")
TEXT = [Texture("maze3D_new/textures/5_secs.png"), Texture("maze3D_new/textures/4secs.png"), Texture("maze3D_new/textures/3secs.png"), 
		Texture("maze3D_new/textures/2secs.png"), Texture("maze3D_new/textures/1secs.png"), Texture("maze3D_new/textures/play.png"), Texture("maze3D_new/textures/end_game.png")]
####################################################################

# (field of view, aspect ratio,near,far)
# control cameraPos, viewMatrix, projection for better 3D view adjusting
cameraPos = pyrr.Vector3([80, 80, 500])
up = pyrr.Vector3([0.0, 1.0, 0.0])
viewMatrix = pyrr.matrix44.create_look_at(cameraPos, pyrr.Vector3([80, 80, 0]), up)
glUniformMatrix4fv(VIEW_LOC, 1, GL_FALSE, viewMatrix)
projection = pyrr.matrix44.create_perspective_projection_matrix(80, 800 / 800, 1, 7000)
glUniformMatrix4fv(PROJ_LOC, 1, GL_FALSE, projection)
glUniform3f(LIGHT_LOC, -400, 200, 300)


layout_up_right = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1, 4, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 5, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]


layout_up_left = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 3, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1, 4, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 5, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]

layout_down_right = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 5, 1, 4, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1, 4, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 5, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 5, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 4, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]


center = [0, 0]
left_down = [-118, -118]
left_up = [-104, 73]
right_down = [73, -104]

# #################
# goal = left_down
# ################
