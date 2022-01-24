function saveMovie(frames, name)
    %% Initialize video
    myVideo = VideoWriter(strcat("C:\Users\zecoe\Desktop\Tese\Matlab\Final\Videos\",  name)); %open video file
    myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
    open(myVideo)

    writeVideo(myVideo, frames);

    close(myVideo)
end