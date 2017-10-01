lport=10000;
udpo=pnet('udpsocket',lport);
%%
nPackets=0;
wrong_packets=false;
PACK_SIZE=4096;

host='127.0.0.1';
port=12345;

while(1)
    
    len=pnet(udpo,'readpacket');
    data=pnet(udpo,'read',5000,'uint8');
    % state 1: receive number of image packets
    if (len ==12)
        ID=typecast(data,'int32');
        disp('got number of packets')
        nPackets=ID(3);
        
        % acknowledge that we got the total number of image packets
        pnet(udpo,'write',typecast(int32(nPackets),'uint8'));              % Write to write buffer
        pnet(udpo,'writepacket',host,port);   % Send buffer as UDP packet
        
        t1=tic;
        counter=0;
        longbuff=uint8(zeros(1,PACK_SIZE*nPackets));
        while (counter < nPackets)
            % we took long time
            if (toc(t1)>10)
                disp('took long time')
                wrong_packets=true;
                break;
            end
            % read one image packet
            len=pnet(udpo,'readpacket');
            data=pnet(udpo,'read',5000,'uint8');

            if len ~= PACK_SIZE
                continue;
            end
            
            disp('i am here')
            
            longbuff((counter*PACK_SIZE+1):(counter*PACK_SIZE+PACK_SIZE))=data;
            counter=counter+1;
            
            % acknowledge that we got one image packet
            pnet(udpo,'write',typecast(int32(999),'uint8'));% Write to write buffer
            pnet(udpo,'writepacket',host,port);   % Send buffer as UDP packet
            
        end
        
        if (~wrong_packets)
            disp('got complete image...')
            % pass longbuff to opencv to decode image
        end
        
    end
    %pause(0.01)
end
%%
pnet(udpo,'close');