class Visualizer(object):
    def __init__(self):
        import cv2
        self.cv2 = cv2
        from numpy import zeros

        self.counter = 0
        self.position_graph = zeros((800, 800, 3), dtype = 'uint8')

    def position(self, x, y, z, every = 100):
        if self.counter%every is 0:
            blue = 255 + 50*z# color represents depth
            self.cv2.circle(self.position_graph, (int(10*x) + 400, 800 - int(10*y)), 5, (blue, 255, 0), -1)
            self.cv2.imshow('position', self.position_graph)
            self.cv2.waitKey(1)

        self.counter += 1

def main(simulating = False):
    if simulating:
        from main_module.gyro.simulation import Simulated as Gyro
        from main_module.odometer.simulation import Simulated as Odometer
        from main_module.propulsion.simulation import Simulated as Propulsion
        from main_module.planning.s2018 import coach

        gyro = Gyro(30.0)
        odometer = Odometer(30.0)
        propulsion = Propulsion(gyro, 30.0)

        strategy = coach.list_of_strategies()[0]
        strategy = coach.initialize(strategy, gyro, odometer)

        visualizer = Visualizer()

        while True:
            try:
                strategy.run(propulsion, [])
                # print(odometer.position.xyz)
                visualizer.position(odometer.position.x, odometer.position.y, odometer.position.z)
            except KeyboardInterrupt:
                break

    else:
        from main_module.gyro.ros_gyro import ROS_Gyro as Gyro
        from main_module.odometer.ros_odometer import ROS_Odometer as Odometer
        from main_module.propulsion.robot2018 import Robot2018 as Propulsion


if __name__ == '__main__':
    main(simulating = True)