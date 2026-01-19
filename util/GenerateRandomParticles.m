
function particles = GenerateRandomParticles(num, spread)
    particles = (rand(3, num) - 0.5) * 2 * spread;
end