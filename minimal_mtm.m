classdef minimal_mtm < dynamicprops
    % Class used to interface with an MTM,

    % values set by this class, can be read by others
    properties (Access = protected)
        crtk_utils;
        ral;
        ros_namespace;
        % publishers
        gravity_compensation_publisher;
        % message placeholders
        std_msgs_Bool;
        cleanup;
    end

    properties (SetAccess = immutable)
        body;
        spatial;
        local;
    end

    methods

        function [name] = name(self)
            name = self.ros_namespace;
        end

        function self = minimal_mtm(name)
            self.ros_namespace = name;
            self.ral = crtk.ral('gc_estimate');
            self.crtk_utils = crtk.utils(self, name, self.ral);
            % operating state
            self.crtk_utils.add_operating_state();
            % joint space
            self.crtk_utils.add_measured_js();
            self.crtk_utils.add_setpoint_js();
            self.crtk_utils.add_servo_jf();
            self.crtk_utils.add_move_jp();
            % custom publishers
            topic = strcat(self.ros_namespace, '/use_gravity_compensation');
            self.gravity_compensation_publisher = ...
                self.ral.publisher(topic, rostype.std_msgs_Bool);
            % one time creation of messages to prevent lookup and creation at each call
            self.std_msgs_Bool = self.ral.message(rostype.std_msgs_Bool);

            self.cleanup = onCleanup(@()delete(self));

            % hacks to wait for ROS 2 to be ready
            pause(5);
            self.home();
            self.home();
            disp('pausing 5 seconds to make sure all subscribers are working');
            pause(5)
        end

        function delete(self)
            delete(self.crtk_utils);
            delete(self.ral);
        end

        function result = use_gravity_compensation(self, gravity)
            self.std_msgs_Bool.data = gravity;
            % send message
            send(self.gravity_compensation_publisher, ...
                 self.std_msgs_Bool);
            result = true;
        end

    end

end
