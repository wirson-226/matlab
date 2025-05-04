% modules/init_video.m
function vwriter = init_video(dt)
    vwriter = VideoWriter('formation_animation.mp4','MPEG-4');
    vwriter.FrameRate = round(1/dt);
    vwriter.Quality = 100;  % 提高质量
    open(vwriter);
end