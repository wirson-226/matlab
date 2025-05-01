% modules/init_video.m
function vwriter = init_video(dt)
    vwriter = VideoWriter('formation_animation.mp4','MPEG-4');
    vwriter.FrameRate = round(1/dt);
    open(vwriter);
end