function getRigidBodiesfromMocap( ~ , event )
	%clc
    global  positionNED rotationNED nrb
    %event.data.OtherMarkers(1)
    % number of available rigid bodies
    nrb=event.data.nRigidBodies;
    
	%frame = evnt.data.iFrame;
	
    position=struct('x', cell(nrb,1),...
                       'y', cell(nrb,1),...
                       'z', cell(nrb,1));
    positionNED=struct('x', cell(nrb,1),...
                       'y', cell(nrb,1),...
                       'z', cell(nrb,1));
    for i=1:nrb
        rb = event.data.RigidBodies( i );
        xyz=double([rb.x, rb.y, rb.z]); % in meters
        
        position(i).x=xyz(1);
        position(i).y=xyz(2);
        position(i).z=xyz(3);
    

        rbqx = rb.qx;
        rbqy = rb.qy;
        rbqz = rb.qz ;
        rbqw = rb.qw;

        q = quaternion( rbqx, rbqy, rbqz, rbqw );
        qRot = quaternion( 0, 0, 0, 1);
        q = mtimes( q, qRot);
        a = EulerAngles( q , 'zyx' );
        rx = a( 1 ) * -180.0 / pi;
        %ry = a( 2 ) * -180.0 / pi;
        % should be
        ry = a( 2 ) * 180.0 / pi;

        %rz = a( 3 ) * 180.0 / pi;
        rz = a( 3 ) * -180.0 / pi;
        rotation = [ rx , ry , rz ];

        % position in NED in meters
        positionNED(i).x=double(rb.z); positionNED(i).y=double(-rb.x); positionNED(i).z=double(-rb.y);
        % Quaternion in NED in degrees
        q_wxyz.w=rbqw; q_wxyz.x=rbqz; q_wxyz.y=-rbqx; q_wxyz.z=-rbqy;

        % attitude angles in NED
        qNED = quaternion( rbqw, rbqz, -rbqx, -rbqy );
        aNED = EulerAngles( qNED , '123' );
        rotationNED=[aNED(1), aNED(2), aNED(3)]*180/pi;
        
    end
%     % set the MatMav properties
%     Mav1.setMocap.x=positionNED.x;
%     Mav1.setMocap.y=positionNED.y;
%     Mav1.setMocap.z=positionNED.z;
%     Mav1.setMocap.qw=q_wxyz.w;
%     Mav1.setMocap.qx=q_wxyz.x;
%     Mav1.setMocap.qy=q_wxyz.y;
%     Mav1.setMocap.qz=q_wxyz.z;
    
end
