function [pixel, is_visible] = ProjectPoint(X_world, K, R_C_W, t_W_C, img_w, img_h)
    t_C_W = -R_C_W * t_W_C;
    X_cam = R_C_W * X_world + t_C_W;
    
    if X_cam(3) <= 0.001
        pixel = [-1;-1];
        is_visible = false;
        return;
    end
    
    pixel_homo = K * X_cam;
    pixel = pixel_homo(1:2) / pixel_homo(3);
    
    is_visible = (pixel(1) >= 0 && pixel(1) <= img_w && pixel(2) >= 0 && pixel(2) <= img_h);
end