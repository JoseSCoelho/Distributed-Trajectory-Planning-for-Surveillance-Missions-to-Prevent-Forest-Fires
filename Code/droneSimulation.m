function [drone, gmPDF] = droneSimulation(drone, gmPDF, waypoint)
    ref = inv([1 0 0 0; 0 1 0 0]*inv(eye(4)-drone.A+drone.B*drone.K)*drone.B)*[waypoint.x;waypoint.y];
    u = ref - drone.K*drone.State;
    u(1) = min(u(1), 1); u(1) = max(u(1), -1.5);
    u(2) = min(u(2), 1); u(2) = max(u(2), -1.5);
    drone.State = drone.A*drone.State + drone.B*u;
    drone.u = u;
end