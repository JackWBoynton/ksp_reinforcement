import sys
import krpc
import time

print(dir(krpc.error))
conn = krpc.connect(name="testing")
vessel = conn.space_center.active_vessel
waypoints = conn.space_center.waypoint_manager
bodies = conn.space_center.bodies
flight_info = vessel.flight
control_info = vessel.control
resources_info = vessel.resources
sit = vessel.situation

altitude = self.conn.add_stream(getattr, self.flight_info, 'surface_altitude')
latitude = self.conn.add_stream(getattr, self.flight_info, 'latitude')
longitude = self.conn.add_stream(getattr, self.flight_info, 'longitude')
h_speed = self.conn.add_stream(getattr, self.flight_info, 'horizontal_speed')
v_speed = self.conn.add_stream(getattr, self.flight_info, 'vertical_speed')
#rot = self.conn.add_stream(getattr, self.flight_info, "rotation")
pointing = self.conn.add_stream(getattr, self.flight_info, "direction")
#obt_normal = self.conn.add_stream(getattr, self.flight_info, "normal")
#aero = self.conn.add_stream(getattr, self.flight_info, "aerodynamic_force")
#aoa = self.conn.add_stream(getattr, self.flight_info, "angle_of_attack")
#slid_slip_a = self.conn.add_stream(getattr, self.flight_info, "sideslip_angle")
#drag_coeff = self.conn.add_stream(getattr, self.flight_info, "drag_coefficient")
#lift_coeff = self.conn.add_stream(getattr, self.flight_info, "lift_coefficient")

max_thrust = self.conn.add_stream(getattr, self.vessel, "available_thrust")
#pos = self.conn.add_stream(getattr, self.vessel, "surface_reference_frame")

legs = self.conn.add_stream(getattr, self.control_info, "legs") # can be set
throttle = self.conn.add_stream(getattr, self.control_info, "throttle") # can be set
pitch = self.conn.add_stream(getattr, self.control_info, "pitch") # can be set
yaw = self.conn.add_stream(getattr, self.control_info, "yaw") # can be set
roll = self.conn.add_stream(getattr, self.control_info, "roll") # can be set

fuel = self.conn.add_stream(self.resources_info.amount, "LiquidFuel")

waypoints.add_waypoint(-0.09711481043049347-0.5, -74.55776907641479, bodies["Kerbin"], "Landing Site")



while True:
    print(vessel.situation, v(), max_thrust(), rot())
    time.sleep(0.5)
