import numpy as np
import pygame
# from pygame.locals import *

COLORS = dict(
    w = (255, 255, 255),
    k = (0, 0, 0),
    g = (0, 255, 0),
    r = (255, 0, 0),
    b = (0, 0, 255)
)

class PygameWindow:
    def __init__(self,
                 name,
                 size,
                 real_map_size_pixels,
                 map_settings_dict,
                 goal_point,
                 stopping_dist):

        pygame.init()
        pygame.display.set_caption(name)

        self.size = size
        self.meters_per_pixel = map_settings_dict['resolution'] / self.size[0] * real_map_size_pixels[0]
        self.map_settings_dict = map_settings_dict
        self.origin = np.array(map_settings_dict['origin'])

        map_img = pygame.image.load('../maps/willowgarageworld_05res.png')
        map_img = pygame.transform.scale(map_img, self.size)

        self.screen = pygame.display.set_mode(self.size)
        self.screen.blit(map_img, (0, 0))
        pygame.display.flip()

        full_map_height = map_settings_dict['resolution'] * real_map_size_pixels[1]
        # 80 + -49.25 since origin is relative to bottom left corner, but pygame 0, 0 is top left corner
        self.origin_pixels = np.array([-self.origin[0], full_map_height + self.origin[1]]) / self.meters_per_pixel

        self.add_se2_pose([0, 0, 0], length=5, color=COLORS['r'])
        self.add_point(goal_point.flatten(), radius=stopping_dist / self.meters_per_pixel, color=COLORS['g'])

    def add_point(self, map_frame_point, radius=1, width=0, color=COLORS['k']):
        map_frame_point[1] = -map_frame_point[1]  # for top left origin
        point_vec = self.point_to_vec(np.array(map_frame_point) / self.meters_per_pixel + self.origin_pixels)
        pygame.draw.circle(self.screen, color, point_vec, radius, width)
        pygame.display.update()

    def add_se2_pose(self, map_frame_pose, length=1, width=0, color=COLORS['k']):
        map_frame_pose[1] = -map_frame_pose[1]  # for top left origin
        l = length
        p_center = np.array(map_frame_pose[:2]) / self.meters_per_pixel + self.origin_pixels
        theta = map_frame_pose[2]

        # y terms all made opposite of expected here because of top left origin
        p_back = np.array([-l * np.cos(theta) + p_center[0], l * np.sin(theta) + p_center[1]])
        p_1 = np.array([-l/2 * np.sin(theta) + p_back[0], -l/2 * np.cos(theta) + p_back[1]])
        p_2 = np.array([l/2 * np.sin(theta) + p_back[0], l/2 * np.cos(theta) + p_back[1]])

        c_vec = self.point_to_vec(p_center)
        p1_vec = self.point_to_vec(p_1)
        p2_vec = self.point_to_vec(p_2)

        pygame.draw.polygon(self.screen, color, [c_vec, p1_vec, p2_vec], width=width)
        pygame.display.update()

    # def add_line(self, p1, p2, width=1, color=COLORS['k']):
    #     pygame.draw.line(self.screen, color, p1, p2, width)

    # def remove_line(self, p1, p2, width=1, color=COLORS['w']):
    #     pygame.draw.line(self.screen, color, p1, p2, width)

    def point_to_vec(self, point):
        vec = pygame.math.Vector2()
        vec.xy = point
        return vec

    def check_for_close(self):
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Closing planner.")