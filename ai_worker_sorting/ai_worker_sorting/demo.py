from .core import DryRobot, Point, pick_and_place


def main():
    pick_and_place(
        DryRobot(), Point(0.40, 0.10, 0.15), Point(0.35, -0.20, 0.20),
        Point(0.30, 0.0, 0.40), [0.1, 0.7, -0.4, 0.4, 0.05, 0.7],
        clearance=0.10, grasp_offset=0.0, check=lambda: None, report=print,
    )


if __name__ == '__main__':
    main()
