import numpy as np


class Controller():
    def __init__(self):
        self.controller = "p" # or pd
        
        self.k_p_theta = 10.0
        self.k_p_dist = 20.0
        self.k_d_dist = 500.0
        self.last_dist = np.Inf
        self.v = 0.5
        self.theta_threshold = np.pi / 6
        pass

    def angle_control_commands(self, dist, angle):
        # Return the angular velocity in order to control the Duckiebot so that it follows the lane.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        # Outputs:
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        if self.controller == "p":
            k_p_d = self.k_p_theta ** 2  / (4 * self.v)
            d_threshold = np.abs(self.k_p_theta * self.theta_threshold / (k_p_d + np.exp(-6)))
            d = min(max(dist, -d_threshold), d_threshold)
            omega = k_p_d * dist + self.k_p_theta * angle
        elif self.controller == "pd":
            ddist = 0
            if self.last_dist != np.Inf:
                d_dist = dist - self.last_dist
            omega = self.k_p_dist * dist + self.k_d_dist * d_dist
            self.last_dist = dist
        
        return  omega

    def pure_pursuit(self, env, pos, angle, follow_dist=0.25):
        # Return the angular velocity in order to control the Duckiebot using a pure pursuit algorithm.
        # Parameters:
        #     env: Duckietown simulator
        #     pos: global position of the Duckiebot
        #     angle: global angle of the Duckiebot
        # Outputs:
        #     v: linear veloicy in m/s.
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        
        closest_curve_point = env.unwrapped.closest_curve_point
        
        # Find the curve point closest to the agent, and the tangent at that point
        closest_point, closest_tangent = closest_curve_point(pos, angle)

        iterations = 0
        
        lookup_distance = follow_dist
        multiplier = 0.5
        curve_point = None
        
        while iterations < 10:            
            ########
            #
            #TODO 1: Modify follow_point so that it is a function of closest_point, closest_tangent, and lookup_distance
            #
            ########
            follow_point = closest_point + closest_tangent * lookup_distance
            
            curve_point, _ = closest_curve_point(follow_point, angle)
            
            # If we have a valid point on the curve, stop
            if curve_point is not None:
                break
            iterations += 1
            lookup_distance *= multiplier
        ########
        #
        #TODO 2: Modify omega
        #
        ########
        L = curve_point - pos
        cos = np.cos(angle)
        sin = np.sin(angle)
        rot = np.array([[cos, -sin], [sin, cos]])
        L_r = np.dot([L[2], L[0]], rot)
        direction = np.arctan2(L_r[0] / np.linalg.norm(L_r), L_r[1] / np.linalg.norm(L_r))
        
        v = 0.5
        omega = -2 * v * np.sin(direction) / np.linalg.norm(L)

        return v, omega