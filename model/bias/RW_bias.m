function beta_dot = RW_bias(sigma)
    eta = sigma * randn();
    beta_dot = eta;
end