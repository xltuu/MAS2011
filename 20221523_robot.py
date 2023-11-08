import pygame
import numpy as np

RED = (255, 0, 0)
FPS = 30   # frames per second

WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 800


def Rmat(degree):
    rad = np.deg2rad(degree)
    c = np.cos(rad)
    s = np.sin(rad)
    R = np.array([[c, -s, 0],
                  [s,  c, 0], [0, 0, 1]])
    return R


def Tmat(tx, ty):
    Translation = np.array([
        [1, 0, tx],
        [0, 1, ty],
        [0, 0, 1]
    ])
    return Translation
#


def draw(P, H, screen, color=(100, 200, 200)):
    R = H[:2, :2]
    T = H[:2, 2]
    Ptransformed = P @ R.T + T
    pygame.draw.polygon(screen, color=color,
                        points=Ptransformed)
    return
#


def makeArm(H0, w, h, jointangle, screen, totalarm):
    if totalarm == 0:
        H1 = H0 @ Tmat(w/2, 0)
    else:
        H1 = H0 @ Tmat(w, 0) @ Tmat(0, h/2)
    x, y = H1[0, 2], H1[1, 2]  # joint position
    if totalarm == 0:
        H11 = H1 @ Rmat(-90) @ Tmat(0, -h/2)
    else:
        H11 = H1 @ Rmat(jointangle) @ Tmat(0, -h/2)

    H12 = H11 @ Tmat(0, h/2) @ Rmat(jointangle) @ Tmat(0, -h/2)

    return H12


def main():
    pygame.init()  # initialize the engine

    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()

    w = 200
    h = 33
    X = np.array([[0, 0], [w, 0], [w, h], [0, h]])
    gw = 30
    gh = 10
    Y = np.array([[0, 0], [gw, 0], [gw, gh], [0, gh]])
    position = [WINDOW_WIDTH/2, WINDOW_HEIGHT - 100]
    jointangle1 = 10
    jointangle2 = -20
    jointangle3 = -25
    grangle = 0

    done = False
    totalSpace = 0
    while not done:
        #  input handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
                # Get the state of all keys
        keys = pygame.key.get_pressed()
        if keys[pygame.K_1]:
            if keys[pygame.K_LEFT]:
                jointangle1 -= 10
            elif keys[pygame.K_RIGHT]:
                jointangle1 += 10
        if keys[pygame.K_2]:
            if keys[pygame.K_LEFT]:
                jointangle2 -= 10
            elif keys[pygame.K_RIGHT]:
                jointangle2 += 10
        elif keys[pygame.K_3]:
            if keys[pygame.K_LEFT]:
                jointangle3 -= 10
            elif keys[pygame.K_RIGHT]:
                jointangle3 += 10

        elif keys[pygame.K_SPACE]:
            # grangle -= 10
            if (totalSpace == 0):
                grangle -= 10
                totalSpace += 1
            elif (totalSpace % 2 != 0):
                grangle += 20
                totalSpace += 1
            elif (totalSpace % 2 == 0):
                grangle -= 20
                totalSpace += 1

        # drawing
        screen.fill((200, 254, 219))

        # base
        H0 = Tmat(position[0], position[1]) @ Tmat(0, -h)
        draw(X, H0, screen, (0, 0, 0))  # base

        # arm 1
        H12 = makeArm(H0, w, h, jointangle1, screen, totalarm=0)
        draw(X, H12, screen, (102, 102, 102))  # arm 1, 90 degree

        # arm 2
        H22 = makeArm(H12, w, h, jointangle2, screen, totalarm=1)
        draw(X, H22, screen, (153, 153, 153))

        # arm3
        H32 = makeArm(H22, w, h, jointangle3, screen, totalarm=2)
        draw(X, H32, screen, (204, 204, 204))

        # gripper
        G1 = H32@Tmat(w, 0)@Tmat(0, h/2-gh/2)
        x, y = G1[0, 2], G1[1, 2]
        # G11 = G1@Rmat(jointangle3)@Tmat(0, -h/2)
        draw(Y, G1, screen, (102, 102, 153))

        G2 = G1@Tmat(gw, 0)@Tmat(0, gh)@Tmat(0, gw/2-gh/2)@Rmat(-90)
        draw(Y, G2, screen, (102, 102, 153))

        # gripper's arm
        G3 = G1@Tmat(gw, 0)@Tmat(0, gw/2)@Rmat(grangle)
        draw(Y, G3, screen, (102, 102, 153))

        R_x = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
        G4 = G1@Tmat(gw, 0)@Tmat(0, gw/2) @ R_x @ Tmat(0,
                                                       gw/2) @ Tmat(0, gh/2)@Rmat(grangle)
        draw(Y, G4, screen, (102, 102, 153))

        font = pygame.font.SysFont('FixedSys', 30, True, False)
        text1 = font.render(
            f"1:1st arm   2:2nd arm    3:3rd arm    space:gripper", True, (0, 0, 0))
        text2 = font.render(f"20221523_robotarm", True, (0, 0, 0))
        screen.blit(text1, [10, 10])
        screen.blit(text2, [10, 40])
        # finish
        pygame.display.flip()
        clock.tick(FPS)
    # end of while
# end of main()


if __name__ == "__main__":
    main()
