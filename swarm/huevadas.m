
drone_x = positionDroneNow(1,:) + obj.drone_state.currentState.positionNed.y;
            drone_y = positionDroneNow(2,:) + obj.drone_state.currentState.positionNed.x;
            drone_z = positionDroneNow(3,:) - obj.drone_state.currentState.positionNed.z;
            %! Affichage
%             delete(obj.affichage.drone.model)
            obj.affichage.drone.model = scatter3(drone_x,drone_y,drone_z,'.','k');
            