import pygame
import numpy as np
from parameters import RED, IMG_SIZE, SCREEN_SIZE,cv2,np,RAD2DEG


class Display:
    # Parameters
    SCREEN = None

    LEFT_MARGIN = 5
    TOP_MARGIN = 0
    INTER_LINE = 15

    FONT_PANEL_INFO = None

    # global Variables
    pos_img_in_screen = 0
    current_line = TOP_MARGIN
    log_dict = {}

    # signals

    # methods

    @classmethod
    def _log(cls, title, value):
        """Log marker info at a smaller font size."""
        next_line = cls.current_line + cls.INTER_LINE
        position = (cls.LEFT_MARGIN, next_line)
        cls.log_dict[title] = {"pos": position, 'value': value}
        cls.current_line = next_line

    @classmethod
    def _update_log(cls):
        for title, item in cls.log_dict.items():
            text = f"{title} {item['value']}"
            panel_info = cls.FONT_PANEL_INFO.render(text, True, RED)
            cls.SCREEN.blit(panel_info, item['pos'])

    @classmethod
    def setup(cls):
        # Init pygame
        pygame.init()
        pygame.font.init()  # The font
        cls.FONT_PANEL_INFO = pygame.font.Font('freesansbold.ttf', 14)

        # create pygame screen
        shift_left = SCREEN_SIZE[0] - IMG_SIZE[0]
        cls.pos_img_in_screen = (shift_left, 0)
        cls.SCREEN = pygame.display.set_mode(SCREEN_SIZE)

    @classmethod
    def stop(cls):
        pass

    @classmethod
    def run(cls, frame, markers):

        # Clear the display each frame
        cls.SCREEN.fill((0, 0, 0))  # Fill screen with black
        # Clear the log for each run to prevent overlapping text
        cls.log_dict.clear()
        cls.current_line = cls.TOP_MARGIN

        for idx, marker in enumerate(markers):
            if marker.id == -1:
                continue  # Skip if marker is not valid

            # Log marker information for on-screen display
            cls._log(f"M {idx} ID", marker.id)
            cls._log(f"M {idx} H_angle", int(marker.h_angle* RAD2DEG))
            cls._log(f"M {idx} v_angle", int(marker.v_angle* RAD2DEG))
            cls._log(f"M {idx} m_angle", int(marker.m_angle* RAD2DEG))
            cls._log(f"M {idx} m_distance", marker.m_distance)
            cls._log(f"M {idx} height", marker.height)
            cls._log(f"M {idx} width", marker.width)

        # Draw the video feed on the `pygame` screen
        frame = np.rot90(frame)
        frame = np.flipud(frame)
        frame = pygame.surfarray.make_surface(frame)
       
        cls.SCREEN.blit(frame, cls.pos_img_in_screen)

        # Render the logs on the screen
        cls._update_log()

        # Update the display
        pygame.display.update()
