#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>

#define NUM_BOIDS 200
#define WIDTH 800
#define HEIGHT 600

#define VISUAL_RANGE 75
#define PROTECTED_RANGE 20

#define CENTERING_FACTOR 0.005f
#define AVOID_FACTOR 0.05f
#define MATCHING_FACTOR 0.05f
#define TURN_FACTOR 1.0f

#define MIN_SPEED 10.0f
#define MAX_SPEED 40.0f

#define MAX_BIAS 0.25f
#define BIAS_INCREMENT 0.005f

struct Boid {
    float x, y;
    float vx, vy;
    float biasval;
    int scout_group; // 0: no bias, 1: right, 2: left
};

float randf(float min, float max) {
    return min + static_cast<float>(rand()) / RAND_MAX * (max - min);
}

float clamp(float value, float min, float max) {
    return std::fmax(min, std::fmin(value, max));
}

void update_boids(std::vector<Boid>& boids, float deltaTime) {
    for (auto& boid : boids) {
        float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0;
        int neighboring_boids = 0;
        float close_dx = 0, close_dy = 0;

        for (const auto& other : boids) {
            if (&boid == &other) continue;
            float dx = boid.x - other.x;
            float dy = boid.y - other.y;

            if (std::abs(dx) < VISUAL_RANGE && std::abs(dy) < VISUAL_RANGE) {
                float dist_squared = dx*dx + dy*dy;

                if (dist_squared < PROTECTED_RANGE*PROTECTED_RANGE) {
                    close_dx += dx;
                    close_dy += dy;
                } else if (dist_squared < VISUAL_RANGE*VISUAL_RANGE) {
                    xpos_avg += other.x;
                    ypos_avg += other.y;
                    xvel_avg += other.vx;
                    yvel_avg += other.vy;
                    neighboring_boids++;
                }
            }
        }

        if (neighboring_boids > 0) {
            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;
            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;

            boid.vx += (xpos_avg - boid.x) * CENTERING_FACTOR + (xvel_avg - boid.vx) * MATCHING_FACTOR;
            boid.vy += (ypos_avg - boid.y) * CENTERING_FACTOR + (yvel_avg - boid.vy) * MATCHING_FACTOR;
        }

        boid.vx += close_dx * AVOID_FACTOR * deltaTime;
        boid.vy += close_dy * AVOID_FACTOR * deltaTime;

        // Boundary turn
        if (boid.x < 0) boid.vx += TURN_FACTOR;
        if (boid.x > WIDTH) boid.vx -= TURN_FACTOR;
        if (boid.y < 0) boid.vy += TURN_FACTOR;
        if (boid.y > HEIGHT) boid.vy -= TURN_FACTOR;

        // Bias dynamics
        if (boid.scout_group == 1) {
            if (boid.vx > 0) boid.biasval = std::min(MAX_BIAS, boid.biasval + BIAS_INCREMENT);
            else boid.biasval = std::max(BIAS_INCREMENT, boid.biasval - BIAS_INCREMENT);
        } else if (boid.scout_group == 2) {
            if (boid.vx < 0) boid.biasval = std::min(MAX_BIAS, boid.biasval + BIAS_INCREMENT);
            else boid.biasval = std::max(BIAS_INCREMENT, boid.biasval - BIAS_INCREMENT);
        }

        // Apply bias
        if (boid.scout_group == 1) {
            boid.vx = (1 - boid.biasval)*boid.vx + boid.biasval;
        } else if (boid.scout_group == 2) {
            boid.vx = (1 - boid.biasval)*boid.vx - boid.biasval;
        }

        // Speed control
        float speed = std::sqrt(boid.vx*boid.vx + boid.vy*boid.vy);
        if (speed < MIN_SPEED || speed > MAX_SPEED) {
            boid.vx = (boid.vx / speed) * clamp(speed, MIN_SPEED, MAX_SPEED);
            boid.vy = (boid.vy / speed) * clamp(speed, MIN_SPEED, MAX_SPEED);
        }

        boid.x += boid.vx * deltaTime;
        boid.y += boid.vy * deltaTime;
    }
}

int main() {
    sf::Clock clock;
    sf::RenderWindow window(sf::VideoMode({WIDTH, HEIGHT}), "Boids Simulation - SFML");

    std::vector<Boid> boids;
    for (int i = 0; i < NUM_BOIDS; i++) {
        Boid b;
        b.x = randf(0, WIDTH);
        b.y = randf(0, HEIGHT);
        b.vx = randf(-2, 2);
        b.vy = randf(-2, 2);
        b.biasval = 0.0f;
        b.scout_group = (i < 10) ? 1 : (i < 20) ? 2 : 0; // primi 10 bias destra, successivi 10 bias sinistra
        boids.push_back(b);
    }

    sf::CircleShape shape(4);
    shape.setFillColor(sf::Color::White);
    shape.setOrigin({2, 2});

    while (window.isOpen()) {
        sf::Time dt = clock.restart();
        float deltaTime = dt.asSeconds();

        while (const std::optional event = window.pollEvent())
        {
            // Close window: exit
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // Clear screen
        window.clear();

        update_boids(boids, deltaTime);

        window.clear();
        for (auto& b : boids) {
            shape.setPosition({b.x, b.y});
            window.draw(shape);
        }
        window.display();
    }
    return 0;
}