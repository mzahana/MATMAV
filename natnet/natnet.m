% Copyright NaturalPoint 2015
% Not for Redistribution
% Wrapper class for the NatNetML assembly members
% Matlab 2014b or later

classdef natnet < handle
	properties ( Access = public )
		ClientIP
		ConnectionType
		HostIP
		IsReporting
	end % properties ( Access = public )
	
	
	properties ( SetAccess = private )
		FrameRate
		IsConnected
		Version
	end % properties ( SetAccess = private )

	
	properties ( Access = private )
		Assembly
		AssemblyPath
		AssemblyVersion
		Client
		IsAssemblyLoaded
		IsInitialized
		LastAssemblyPathFile
		MaxListeners
		Listener
		CReattempt
	end % properties ( Access = private )
	
	
	properties ( Access = private , Dependent = true )
		iConnectionType
	end % properties ( Access = private , Dependant = true )
	
	
	methods
		function set.HostIP( obj , val )
			validIP = checkip( obj , val);
			if ( ~isempty( validIP ) )
				obj.disconnect
				obj.HostIP = validIP;
				obj.report( [ 'set host ip address: ' , validIP ] )
			end
		end % set.HostIP
		
		
		function set.ClientIP( obj , val )
			validIP = obj.checkip( val );
			if ( ~isempty( validIP ) )
				disconnect( obj )
				obj.ClientIP = validIP;
				obj.report( [ 'set client ip address: ' , validIP ] )
			end
		end % set.ClientIP
		
		
		function set.ConnectionType( obj , val )
			obj.disconnect
			if ( strcmpi( 'Multicast' , val ) == 1 )
				obj.ConnectionType = 'Multicast';
				obj.report( 'set multicast' )
			elseif ( strcmpi( 'Unicast' , val ) == 1 )
				obj.ConnectionType = 'Unicast';
				obj.report( 'set unicast' )
			else
				obj.report( 'invalid connection type' )
			end
		end % set.ConnectionType
		
		
		function set.IsReporting( obj , val )
			if ( val == 1 )
				obj.IsReporting = 1;
				obj.report( 'reporting enabled' )
			elseif ( val == 0 )
				obj.report( 'reporting disabled' )
				obj.IsReporting = 0;
			end
		end % set.IsReporting
		
		
		function iconnectiontype = get.iConnectionType( obj )
			if ( strcmpi( 'Multicast' , obj.ConnectionType ) == 1 )
				iconnectiontype = 0;
			elseif ( strcmpi( 'Unicast' , obj.ConnectionType ) == 1 )
				iconnectiontype = 1;
			end
		end % iconnectiontype
	end % methods ( property set and get methods)
	

	methods ( Access = public  )
		function obj = natnet(  )
			obj.getLastAssemblyPath
			obj.MaxListeners = 255;
			obj.Listener{ obj.MaxListeners , 1 } = [ ];
			obj.FrameRate = 0;
			obj.IsAssemblyLoaded = 0;
			obj.IsInitialized = 0;
			obj.IsConnected = 0;
			obj.IsReporting = 1;
			obj.CReattempt = 2;
			obj.HostIP = '127.0.0.1';
			obj.ClientIP = '127.0.0.1';
			obj.ConnectionType = 'Multicast';
		end % natnet - constructor
		
		
		function setAssemblyPath( obj )
			[ name , path ] =	uigetfile( '*.dll' , 'Select the NatNetML.dll assembly' );
			if ( strcmpi( name , 'NatNetML.dll') == 1 )
				assemblyPath = strcat( path , name );
				fileid = fopen( obj.LastAssemblyPathFile , 'wt');
				fprintf( fileid , '%s' , assemblyPath );
				fclose( fileid );
				if ( obj.IsAssemblyLoaded == 0 )
					obj.AssemblyPath = assemblyPath;
					obj.report( [ 'defined assembly path: ' , assemblyPath ] ) 
				else
					obj.report( 'restart matlab to apply the changes' )
				end
			else
				obj.report( 'invalid assembly path in setAssemblyPath' )
			end
		end % setAssemlbyPath
		
							
		function connect( obj )
			obj.disconnect
			obj.getAssemblies
			if ( obj.IsAssemblyLoaded == 0 )
				if ( isempty( obj.AssemblyPath ) )
					obj.setAssemblyPath
					obj.addAssembly
					obj.getAssemblies
				else
					obj.addAssembly
					obj.getAssemblies
				end	
			end
			if ( obj.IsAssemblyLoaded == 0 )
				report( obj , 'natnetml assembly is missing or undefined' )
				return
			end
			obj.report( 'initializing client' )
			try
				obj.Client = NatNetML.NatNetClientML( obj.iConnectionType );

				v = obj.Client.NatNetVersion(  );
				obj.Version = sprintf( '%d.%d.%d.%d', v( 1 ) , v( 2 ) , v( 3 ) , v( 4 ) );
				if  ( isempty( obj.Client ) == 1 || obj.IsAssemblyLoaded == 0 )
					obj.report( 'client object invalid' )
					return
				else
					obj.report( [ 'natnet version: ' , obj.Version ] )
				end
				flg = obj.Client.Initialize( obj.HostIP , obj.ClientIP );
				if flg == 0
					obj.report( 'initialization succeeded' )
					obj.IsInitialized = 1;
				else
					obj.report( 'initialization failed' )
					obj.IsInitialized = 0;
					return
				end
			catch exception
				rethrow(exception)
			end
			obj.verifyConnection
			obj.getFrameRate;
		end % connect
		
		
		function verifyConnection( obj )
			if ( isempty( obj.Client ) == 1 )
				obj.IsInitialized = 0;
				obj.IsConnected = 0;
				obj.report( 'client object is undefined' );
			else
				[ ~ , retcode ] = obj.Client.SendMessageAndWait( 'FrameRate' );
				if( retcode == 0 )
					obj.IsConnected = 1;
					report( obj , 'client connected' )
				else
					obj.IsConnected = 0;
					obj.FrameRate = 0;
					report( obj , 'unable to connect to host' );
				end
			end
		end % verifyConnection
		
		
		function addlistener( obj , aListenerIndex , functionhandlestring )
			if ~isnumeric( aListenerIndex ) || aListenerIndex < 1 || aListenerIndex > obj.MaxListeners
				obj.report( 'invalid index' );
				return
			end
			
			if isa( functionhandlestring , 'char' ) && ~isempty( which( functionhandlestring ) );
				functionhandle = str2func ( [ '@( src , evnt)' , functionhandlestring , '( src , evnt)' ] );		

				if ~isa( obj.Listener{ aListenerIndex , 1 } , 'event.listener' );
					obj.report( [ 'adding listener in slot:' , num2str(aListenerIndex ) ] );
					obj.Listener{ aListenerIndex , 1 } = addlistener( obj.Client , 'OnFrameReady2' , functionhandle );
					obj.disable( aListenerIndex );
				elseif isa( obj.Listener{ aListenerIndex , 1 } , 'event.listener' )
					obj.report( [ 'replacing listener in slot: ' , num2str(aListenerIndex ) ] )
					obj.disable( aListenerIndex )
					delete( obj.Listener{ aListenerIndex , 1 } )
					obj.Listener{ aListenerIndex , 1 } = addlistener( obj.Client, 'OnFrameReady2' , functionhandle );
					obj.disable ( aListenerIndex )
				end
			else
				obj.report( [ 'invalid function set for slot: ' , num2str( aListenerIndex ) ] )
				return
			end
		end % addlistener
		
		
		function getFrameRate( obj )
			if ( obj.IsConnected == 1 );
				[ bytearray , rc ] = obj.Client.SendMessageAndWait( 'FrameRate' );
				if rc == 0
					bytearray = uint8( bytearray );
					obj.FrameRate =  typecast( bytearray , 'single' );
					obj.report( [ 'frame rate: ' , num2str( obj.FrameRate ) , ' fps' ] )
				else
					obj.FrameRate = 0;
				end
			else
				obj.report( 'connection not established' )
			end
		end % getFrameRate
		
		
		function modelDescription = getModelDescription( obj )
			if ( obj.IsConnected == 1 )
				dataDescriptions = obj.Client.GetDataDescriptions(  );
				modelDescription.TrackingModelCount = dataDescriptions.Count;
				report( obj , [ 'number of trackables: ' , num2str( dataDescriptions.Count ) ] )
				modelDescription.MarkerSetCount = 0;
				modelDescription.RigidBodyCount = 0;
				modelDescription.SkeletonCount = 0;

				for i = 1 : modelDescription.TrackingModelCount
					descriptor = dataDescriptions.Item( i - 1 );
					% marker sets
					if ( descriptor.type == 0)
						modelDescription.MarkerSetCount = modelDescription.MarkerSetCount + 1;
						modelDescription.MarkerSet( modelDescription.MarkerSetCount ).Name = char( descriptor.Name );
						modelDescription.MarkerSet( modelDescription.MarkerSetCount ).MarkerCount = descriptor.nMarkers;
						for k = 1 : descriptor.nMarkers
							modelDescription.MarkerSet( modelDescription.MarkerSetCount ).Markers( k ).Label = char( descriptor.MarkerNames( k ) );
						end
					% rigid bodies
					elseif ( descriptor.type == 1 )
						modelDescription.RigidBodyCount = modelDescription.RigidBodyCount + 1;
						modelDescription.RigidBody( modelDescription.RigidBodyCount ).Name = char( descriptor.Name );
						modelDescription.RigidBody( modelDescription.RigidBodyCount ).ID = descriptor.ID ;
						modelDescription.RigidBody( modelDescription.RigidBodyCount ).ParentID = descriptor.parentID;
						modelDescription.RigidBody( modelDescription.RigidBodyCount ).OffsetX = descriptor.offsetx;
						modelDescription.RigidBody( modelDescription.RigidBodyCount ).OffsetX = descriptor.offsetx;
						modelDescription.RigidBody( modelDescription.RigidBodyCount ).OffsetX = descriptor.offsetx;
						modelDescription.RigidBody( modelDescription.RigidBodyCount ).Type = descriptor.type;						
					% skeletons
					elseif ( descriptor.type == 2 )
						modelDescription.SkeletonCount = modelDescription.SkeletonCount + 1;
						modelDescription.Skeleton( modelDescription.SkeletonCount ).Name = char( descriptor.Name );
						modelDescription.Skeleton( modelDescription.SkeletonCount ).ID = descriptor.ID;
						modelDescription.Skeleton( modelDescription.SkeletonCount ).SegmentCount = descriptor.nRigidBodies;
						for k = 1 : modelDescription.Skeleton( modelDescription.SkeletonCount ).SegmentCount
							modelDescription.Skeleton( modelDescription.SkeletonCount ).Segment( k ).Name = char( descriptor.RigidBodies( k ).Name ); 
							modelDescription.Skeleton( modelDescription.SkeletonCount ).Segment( k ).ID = descriptor.RigidBodies( k ).ID;
							modelDescription.Skeleton( modelDescription.SkeletonCount ).Segment( k ).ParentID = descriptor.RigidBodies( k ).parentID;
							modelDescription.Skeleton( modelDescription.SkeletonCount ).Segment( k ).OffsetX = descriptor.RigidBodies( k ).offsetx;
							modelDescription.Skeleton( modelDescription.SkeletonCount ).Segment( k ).OffsetY = descriptor.RigidBodies( k ).offsety;
							modelDescription.Skeleton( modelDescription.SkeletonCount ).Segment( k ).OffsetZ = descriptor.RigidBodies( k ).offsetz;
							modelDescription.Skeleton( modelDescription.SkeletonCount ).Segment( k ).Type = descriptor.RigidBodies( k ).type;
						end
					else
						report( obj , 'invalid asset type' )
					end
				end
				report( obj , [ 'number of markersets: ' , num2str( modelDescription.MarkerSetCount ) ] )
				report( obj , [ 'number of rigid bodies: ' , num2str( modelDescription.RigidBodyCount ) ] )
				report( obj , [ 'number of skeletons: ' , num2str( modelDescription.SkeletonCount ) ] )
			else
				obj.report( 'connection not established' )
			end
		end % getModelDescription
		
		
		function frameOfMocapData = getFrame( obj , varargin )
			if mod( length( varargin ) , 2 ) ~= 0
				obj.report( 'invalid value/key pair' )
				return
			end
			if ( obj.IsConnected == 1 )
				data = obj.Client.GetLastFrameOfData();
				sFrameOfData.Frame =  uint64(data.iFrame);
				sFrameOfData.Timestamp = double(data.fTimestamp);
				if isempty( varargin )
					sFrameOfData.Timecode = double(data.Timecode);
					sFrameOfData.TimecodeSubframe = double(data.TimecodeSubframe);
					sFrameOfData.UnlabeledMarker = data.OtherMarkers;
					sFrameOfData.LabeledMarker = data.LabeledMarkers;
					sFrameOfData.RigidBody = data.RigidBodies;
					sFrameOfData.Skeleton = data.Skeletons;
					frameOfMocapData = sFrameOfData;
					return
				end
				for i = 1 : 2 : length( varargin )
					switch varargin{ i }
						case 'timecode'
							if varargin{ i + 1 } == 0
								sFrameOfData.Timecode = double(data.Timecode);
							else
								sFrameOfData.Timecode = double(data.Timecode);
								sFrameOfData.TimecodeSubframe = double(data.TimecodeSubframe);
							end
						case 'unlabeled'
							if strcmpi( varargin{ i + 1 } , 'all' )
								sFrameOfData.UnlabeledMarker = data.OtherMarkers;
							elseif isnumeric( varargin{ i + 1 } )
								sFrameOfData.UnlabeledMarker = data.OtherMarkers( varargin{ i + 1 } );
							end
						case 'labeled'
							if strcmpi( varargin{ i + 1 } , 'all' )
								sFrameOfData.LabeledMarker = data.LabeledMarkers;
							elseif isnumeric( varargin{ i + 1 } )
								sFrameOfData.LabeledMarker = data.LabeledMarkers( varargin{ i + 1 } );
							end
						case 'rigidbody'
							if strcmpi( varargin{ i + 1 } , 'all' )
								sFrameOfData.RigidBody = data.RigidBodies;
							elseif isnumeric( varargin{ i + 1 } )
								sFrameOfData.RigidBody = data.RigidBodies( varargin{ i + 1 } );
							end
						case 'skeleton'
							if strcmpi( varargin{ i + 1 } , 'all' )
								sFrameOfData.Skeleton = data.Skeletons;
							elseif isnumeric( varargin{ i + 1 } )
								sFrameOfData.Skeleton = data.Skeletons( varargin{ i + 1 } );
							end
						otherwise
					end
				end
				frameOfMocapData = sFrameOfData;
			else
				obj.report( 'connection not established' )
			end
		end % getFrame
		

		function enable( obj , eListenerIndex )
			if ~isnumeric( eListenerIndex ) || eListenerIndex < 0 || eListenerIndex > obj.MaxListeners
				obj.report( 'invalid index' )
				return
			end
			
			if obj.IsInitialized == 0
				return
			end

			if eListenerIndex == 0
				for k = 1:obj.MaxListeners
					if isa( obj.Listener{ k ,  1} , 'event.listener' ) && obj.Listener{ k ,  1}.Enabled == false
						obj.Listener{ k , 1  }.Enabled = true;
						obj.report( [ 'listener enabled in slot: ' , num2str( k ) ] )
					end
				end
			else
				if( isa( obj.Listener{ eListenerIndex , 1 } , 'event.listener' ) ) && obj.Listener{ eListenerIndex , 1 }.Enabled == false
					obj.Listener{ eListenerIndex , 1 }.Enabled = true;
					obj.report( [ 'listener disabled in slot: ' , num2str( eListenerIndex ) ] )
				end
			end
		end % enable
		

		function disable( obj , dListenerIndex );
			if ~isnumeric( dListenerIndex ) || dListenerIndex < 0 || dListenerIndex > obj.MaxListeners
				obj.report( 'invalid index' );
				return
			end
			
			if obj.IsInitialized == 0;
				return
			end

			if dListenerIndex == 0
				for k = 1:obj.MaxListeners
					if isa( obj.Listener{ k , 1 } , 'event.listener'  )&& obj.Listener{ k ,  1}.Enabled == true;
						obj.Listener{ k }.Enabled = false;
						obj.report( [ 'listener disabled in slot: ' , num2str( k ) ] )
					end
				end
			else
				if( isa( obj.Listener{ dListenerIndex , 1 } , 'event.listener' ) ) && obj.Listener{ dListenerIndex , 1 }.Enabled == true;
					obj.Listener{ dListenerIndex , 1 }.Enabled = false;
					obj.report( [ 'listener disabled in slot: ' , num2str( dListenerIndex ) ] )
				end
			end
		end % disable


		function startRecord( obj )
			if ( obj.IsConnected == 1 )
				for i = 1:obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( 'StartRecording' );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % startRecord


		function stopRecord( obj )
			report( obj , 'sending stop record command' )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( 'StopRecording' );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(1)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % stopRecord

		
		function cycleRecord( obj , iterations , duration , delay )
			if ( isnumeric( iterations ) && isnumeric( duration ) && isnumeric( delay ) );
				for i = 1:iterations
					pause( delay );
					obj.startRecord;
					if ( obj.IsConnected == 0 )
						return
					end
					pause( duration );
					obj.stopRecord;
					if ( obj.IsConnected == 0 )
						return
					end
				end
			end
		end % cycleRecord
			
		
		function liveMode( obj )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( 'LiveMode' );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % liveMode
		
		
		function editMode( obj )
			if ( obj.IsConnected  == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( 'EditMode' );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % editMode
		
		
		function startPlayback( obj )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( 'TimelinePlay' );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % startPlayback
		
		
		function stopPlayback( obj )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( 'TimelineStop' );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % stopPlayback
		
		
		function setPlaybackStartFrame( obj , startFrame )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( [ 'SetPlaybackStartFrame,' , num2str( startFrame ) ] );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % setPlaybackStartFrame
				
		
		function setPlaybackEndFrame( obj , endFrame )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj. CReattempt	
					[ ~ , rc ] = obj.Client.SendMessageAndWait( [ 'SetPlaybackStopFrame,' , num2str( endFrame ) ] );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % setPlaybackEndFrame
		
				
		function setPlaybackLooping( obj , val )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( [ 'SetPlaybackLooping,' , num2str( val ) ] );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % stopPlaybackLooping
		
		
		function setPlaybackCurrentFrame( obj , currentFrame )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj. CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( [ 'SetPlaybackCurrentFrame,' , num2str( currentFrame ) ] );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % stopPlaybackLooping
				
		
		function setTakeName( obj , name )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( strcat( 'SetRecordTakeName,' , name ) );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % setTakeName
		
		
		function setPlaybackTakeName( obj , name )
			if ( obj.IsConnected == 1 )
				for i = 1 : obj.CReattempt
					[ ~ , rc ] = obj.Client.SendMessageAndWait( strcat( 'SetPlaybackTakeName,' , name ) );
					if rc == 0
						return
					else
						obj.report( 'reattempting a failed command' )
						pause(0.2)
					end
				end
				if rc ~= 0
					obj.report( ' failed to send command')
				end
			else
				obj.report( 'connection not established' )
			end
		end % setPlaybackTakeName
		
		
		function disconnect( obj )
			obj.disable( 0 );
			for k = 1 : obj.MaxListeners
				if isa( obj.Listener{ k , 1 } , 'event.listener' )
					delete( obj.Listener{ k , 1 } )
				end
			end
			obj.Listener{ obj.MaxListeners , 1 } = [  ];
			if ( isempty( obj.Client ) == 0 )
				obj.Client.Uninitialize;
				obj.Client = [  ];
				if ( obj.IsConnected == 1 )
					report( obj , 'disconnecting' );
				end
				obj.IsInitialized = 0;
				obj.IsConnected = 0;
			end
		end % disconnect
		
		
		function delete( obj )
			obj.disconnect
		end % delete - destructor
	end % methods ( Access = public )

	
	methods ( Access = private )
		function getLastAssemblyPath( obj )
			pathtomfile = mfilename( 'fullpath' );
			mfile = mfilename(  );
			mfilenamelength = length( mfile );
			foldertomfile = pathtomfile( 1 : end-mfilenamelength );
			obj.LastAssemblyPathFile = strcat( foldertomfile , 'assemblypath.txt' );
			if ( exist( obj.LastAssemblyPathFile , 'file' ) == 2 )
				assemblypath = textread( obj.LastAssemblyPathFile , '%s' );
				assemblypath = strjoin( assemblypath );
				if( exist( assemblypath , 'file' ) == 2)
					obj.AssemblyPath = assemblypath;
				end
			end	
		end % getLastAssemblyPath
		
		
		function getAssemblies( obj )
			obj.IsAssemblyLoaded = 0;
			obj.Assembly = [  ];
			obj.AssemblyVersion = [  ];
			domain = System.AppDomain.CurrentDomain;
			assemblies = domain.GetAssemblies;
			assembly{ assemblies.Length , 1 } = [  ];
			for i = 1:assemblies.Length
				assembly{ i } = assemblies.Get( i-1 );
				obj.Assembly{ i } = char( assembly{ i }.FullName );
				if ( strncmp ( obj.Assembly{ i } , 'NatNetML' , 8 ) )
					obj.IsAssemblyLoaded = 1;
					aver = regexp( obj.Assembly{ i } , '\d+' , 'match' );
					obj.AssemblyVersion = strcat( aver{ 1 } , '.' , aver{ 2 } , '.' , aver{ 3 } , '.' , aver{ 4 } );
					obj.IsAssemblyLoaded = 1;
				end
			end
		end % getAssemblies
				
		
		function addAssembly( obj )
			if ( exist( obj.AssemblyPath , 'file' ) == 2 )
				[ ~ ] = NET.addAssembly( obj.AssemblyPath );
				obj.IsAssemblyLoaded = 1;
			else
				obj.IsAssemblyLoaded = 0;
				report( obj , 'failed to add NatNetML.dll assembly' );
			end
		end % addAssembly
		
		
		function report( obj , message )
			if ( obj.IsReporting == 1 )
				ctime = strsplit( num2str( clock ) );
				disp( [ ctime{ 1 } , '/' , ctime{ 2 } , '/' , ctime{ 3 } , '   ' , ctime{ 4 } , ':' , ctime{ 5 } , ':' , ctime{ 6 } , '   ' , 'natnet - ' , message ] )
			end
		end % report
		
		
		function validIP = checkip( obj , value )
			if ( ischar( value ) && length( value ) < 16 && length( value ) > 6 )
				val = strsplit( value , '.' );
				if ( length( val ) == 4 )
					if all( isstrprop( val{ 1 } , 'digit' ) ) && all( isstrprop( val{ 2 } , 'digit' ) )...
							&& all( isstrprop( val{ 3 } , 'digit' ) ) && all( isstrprop( val{ 4 } , 'digit' ) )...
							&& length( val{ 1 } ) < 4 && length( val{ 2 } ) < 4 &&  length( val{ 3 } ) < 4 && length( val{ 4 } ) < 4 ...
							&& ~isempty( val{ 1 } ) && ~isempty( val{ 2 } ) &&  ~isempty( val{ 3 } ) && ~isempty( val{ 4 } ) ...
							&& str2double( val{ 1 } ) < 256 && str2double( val{ 2 } ) < 256 && str2double( val{ 3 } ) < 256 && str2double( val{ 4 } ) < 256 ...
							&& str2double( val{ 1 } ) >= 0  && str2double( val{ 2 } ) >= 0 && str2double( val{ 3 } ) >= 0 && str2double( val{ 4 } ) >= 0
						validIP = value;
					else
						report( obj , 'invalid string for ip address (e.x. 127.0.0.1)' )
						validIP = [  ];
					end
				else	
					report( obj , 'invalid string for ip address (e.x. 127.0.0.1)' )
					validIP = [  ];
				end
			else
				report( obj , 'invalid string for ip address (e.x. 127.0.0.1)' )
				validIP = [  ];
			end
		end
	end % methods ( Access = private )
end % classdef natnet < handle
