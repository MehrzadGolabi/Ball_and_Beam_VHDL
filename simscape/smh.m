classdef smh
%SMH  Small helpers for building Simscape Multibody models from code.
methods (Static)
    function p = lib(name)
        % library paths (several contain newlines)
        switch name
            case 'world',     p = sprintf('sm_lib/Frames and\nTransforms/World Frame');
            case 'rt',        p = sprintf('sm_lib/Frames and\nTransforms/Rigid\nTransform');
            case 'tsens',     p = sprintf('sm_lib/Frames and\nTransforms/Transform\nSensor');
            case 'mech',      p = sprintf('sm_lib/Utilities/Mechanism\nConfiguration');
            case 'solver',    p = sprintf('nesl_utility/Solver\nConfiguration');
            case 'sl2ps',     p = sprintf('nesl_utility/Simulink-PS\nConverter');
            case 'ps2sl',     p = sprintf('nesl_utility/PS-Simulink\nConverter');
            case 'brick',     p = 'sm_lib/Body Elements/Brick Solid';
            case 'cyl',       p = 'sm_lib/Body Elements/Cylindrical Solid';
            case 'sphere',    p = 'sm_lib/Body Elements/Spherical Solid';
            case 'revolute',  p = 'sm_lib/Joints/Revolute Joint';
            case 'prismatic', p = sprintf('sm_lib/Joints/Prismatic\nJoint');
            case 'pinslot',   p = 'sm_lib/Joints/Pin Slot Joint';
            case 'weld',      p = 'sm_lib/Joints/Weld Joint';
            case 'spherical', p = sprintf('sm_lib/Joints/Spherical\nJoint');
            case 'rackpinion',p = sprintf('sm_lib/Gears and\nCouplings/Gears/Rack and Pinion\nConstraint');
            case 'extforce',  p = sprintf('sm_lib/Forces and\nTorques/External Force\nand Torque');
            otherwise, error('unknown lib %s', name);
        end
    end

    function b = add(sys, kind, name, pos, varargin)
        b = [sys '/' name];
        add_block(smh.lib(kind), b, 'Position', [pos(1) pos(2) pos(1)+50 pos(2)+40]);
        if ~isempty(varargin), set_param(b, varargin{:}); end
    end

    function b = rt(sys, name, pos, xyz_m, axis, deg)
        % rigid transform: translation [m] then rotation about a standard axis
        b = smh.add(sys, 'rt', name, pos, ...
            'TranslationMethod', 'Cartesian', ...
            'TranslationCartesianOffset', mat2str(xyz_m), 'TranslationCartesianOffsetUnits', 'm');
        if nargin >= 5 && strcmp(axis, 'M')
            set_param(b, 'RotationMethod', 'RotationMatrix', 'RotationMatrix', mat2str(deg));
        elseif nargin >= 5 && ~isempty(axis)
            set_param(b, 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', ['+' axis], ...
                'RotationAngle', num2str(deg), 'RotationAngleUnits', 'deg');
        end
    end

    function b = solid(sys, kind, name, pos, dims, massOrDensity, color, varargin)
        % kind 'brick' dims [x y z]; 'cyl' dims [r L]; 'sphere' dims r   (all m)
        % massOrDensity: struct('m',kg) or struct('rho',kg/m^3)
        b = smh.add(sys, kind, name, pos);
        switch kind
            case 'brick',  set_param(b, 'BrickDimensions', mat2str(dims), 'BrickDimensionsUnits', 'm');
            case 'cyl',    set_param(b, 'CylinderRadius', num2str(dims(1)), 'CylinderRadiusUnits', 'm', ...
                                        'CylinderLength', num2str(dims(2)), 'CylinderLengthUnits', 'm');
            case 'sphere', set_param(b, 'SphereRadius', num2str(dims), 'SphereRadiusUnits', 'm');
        end
        if isfield(massOrDensity, 'm')
            set_param(b, 'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Mass', ...
                'Mass', num2str(massOrDensity.m, 8), 'MassUnits', 'kg');
        elseif isfield(massOrDensity, 'I')
            set_param(b, 'InertiaType', 'Custom', 'Mass', num2str(massOrDensity.mc, 8), 'MassUnits', 'kg', ...
                'MomentsOfInertia', mat2str(massOrDensity.I, 8), 'MomentsOfInertiaUnits', 'kg*m^2', ...
                'CenterOfMass', '[0 0 0]', 'ProductsOfInertia', '[0 0 0]');
        else
            set_param(b, 'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Density', ...
                'Density', num2str(massOrDensity.rho), 'DensityUnits', 'kg/m^3');
        end
        set_param(b, 'GraphicType', 'FromGeometry', 'GraphicVisPropType', 'SimpleVisualProperties', ...
            'GraphicDiffuseColor', mat2str(color));
        if ~isempty(varargin), set_param(b, varargin{:}); end
    end

    function c(sys, a, ap, b, bp)
        % connect physical ports. ports given as 'L1','R1','R2',...
        A = get_param(a, 'PortHandles'); B = get_param(b, 'PortHandles');
        add_line(sys, smh.ph(A, ap), smh.ph(B, bp), 'autorouting', 'on');
    end

    function h = ph(P, s)
        n = str2double(s(2:end));
        switch s(1)
            case 'L', h = P.LConn(n);
            case 'R', h = P.RConn(n);
            case 'I', h = P.Inport(n);
            case 'O', h = P.Outport(n);
        end
    end
end
end
