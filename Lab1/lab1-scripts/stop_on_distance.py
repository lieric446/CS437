from picarx import Picarx
from picarx.music import Music

STOP_DISTANCE = 30

music = Music()
music.music_set_volume(20)

def main():
    move_forward = True

    try:
        px = Picarx()

        print(px.ultrasonic.read())
        while move_forward:
            distance = round(px.ultrasonic.read(), 2)
            print("Current distance: ", distance)
            if distance < STOP_DISTANCE:
                px.forward(0)
                music.sound_play('../car-double-horn.wav')
                move_forward = False
            else:
                px.forward(50)

    finally:
        px.forward(0)


if __name__ == "__main__":
    main()
