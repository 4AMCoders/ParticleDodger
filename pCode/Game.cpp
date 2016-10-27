#include <cmath>
#include <cstdlib>
#include <ctime>
#include <math.h>

#include "Game.h"

Game::Game() : start(0), last(0), current(0), good(true), running(false), 
        width(DEFAULT_WIDTH), height(DEFAULT_HEIGHT), 
        particles(std::vector<Particle>())
{
    // Seed the random number generator
    srand(time(0));
    
    // initialize SDL
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        good = false;
        return;
    }
    
    // initialize SDL window
    window = SDL_CreateWindow("Gravity", SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_SHOWN);
    if (window == NULL)
    {
        good = false;
        return;
    }
    
    // initialize SDL renderer
    renderer = SDL_CreateRenderer(window, -1, 
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == NULL)
    {
        good = false;
        return;
    }
    
    // initialize particle texture
    SDL_Surface* bmp = SDL_LoadBMP("particle.bmp");
    if (bmp == NULL)
    {
        good = false;
        return;
    }
    particleTexture = SDL_CreateTextureFromSurface(renderer, bmp);
    SDL_FreeSurface(bmp);
    if (particleTexture == NULL)
    {
        good = false;
        return;
    }
    
    // initialize our particles
    for (int i = 0; i < PARTICLE_COUNT; ++i)
    {
        particles.push_back(randomParticle());
    }
}

Game::~Game()
{
    if (!good)
    {
        std::cout << "SDL Error: " << SDL_GetError() << std::endl;
    }
    if (particleTexture != NULL)
    {
        SDL_DestroyTexture(particleTexture);
    }
    if (renderer != NULL)
    {
        SDL_DestroyRenderer(renderer);
    }
    if (window != NULL)
    {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
}

int Game::operator()()
{
    if (!good)
    {
        return -1;
    }
    running = true;
    SDL_Event event;
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
    start = SDL_GetTicks();
    last = start;
    while (running) // every iteration is 1 frame
    {
        current = SDL_GetTicks();
        while (SDL_PollEvent(&event))
        {
            handleEvent(event);
        }
        update((current - last) / 1000.0);
        render();
        last = current;
    }
    return 0;
}

void Game::update(double dt)
{
    //std::cout << dt << " sec since last frame.\n";

    //Vectors to store all calculations for retrieval during collision calculation
    //and used to register the changes at the very end of update.
    std::vector<Point> nextPosition;
    std::vector<Point> nextVelocity;
    std::vector<Point> nextAcceleration;

    double d = 180.0 / M_PI;

    //Nested for-loops to calculate the acceleration due to every other particle
    //in respect to one particle, for all particles.
    for (int i = 0; i < PARTICLE_COUNT; ++i) {

	double aYNet = 0;
	double aXNet = 0;

	for (int j = 0; j < PARTICLE_COUNT; ++j) {
		if (i != j) {
			
			double xSum = particles[j].getPos().getX() - particles[i].getPos().getX();

			double ySum = particles[j].getPos().getY() - particles[i].getPos().getY();
			double dist = sqrt((xSum * xSum) + (ySum * ySum));
			double angle = atan2 (ySum, xSum);
			double acceleration = G * particles[j].getMass() / (dist * dist);
			double aX = acceleration * cos(angle * d);
			double aY = acceleration * sin(angle * d);

			aXNet += aX;
			aYNet += aY;
		}
	}

	//Next position is to be calculated by position formula.
	//Next position is (xF, yF).
	double vX = particles[i].getV().getX();
	double vY = particles[i].getV().getY();

	double pX = particles[i].getPos().getX();
	double pY = particles[i].getPos().getY();
	double xF = pX + (vX * dt) + (0.5 * aXNet * dt * dt);
	double yF = pY + (vY * dt) + (0.5 * aYNet * dt * dt);

	double radius = particles[i].getRadius();
	
	//Checking if hitting outside of the box, X - coordinates
	if (xF + radius >= DEFAULT_WIDTH || xF - radius <= 0) {
		vX *= -1;
		if (xF - radius <= 0) {
			xF = radius;
		} else if (xF + radius >= DEFAULT_WIDTH) {
			xF = DEFAULT_WIDTH - radius;
		}
	}

	//Checking if hitting outside of the box, Y - coordinates
	if (yF + radius >= DEFAULT_HEIGHT || yF - radius <= 0) {
		vY *= -1;
		if (yF - radius <= 0) {
			yF = radius;
		} else if(yF + radius >= DEFAULT_HEIGHT) {
			yF = DEFAULT_HEIGHT - radius;
		}
	}

    //Updating the vectors holding our data for bounce and initial calculations.
		nextPosition.push_back(Point(xF, yF));
		nextVelocity.push_back(Point(vX, vY));
		nextAcceleration.push_back(Point(aXNet, aYNet));
    }
	//collisions variable used to prevent double calculating a collision.
	bool collisions[PARTICLE_COUNT][PARTICLE_COUNT];

	for (int i = 0; i < PARTICLE_COUNT; ++i) {
		for (int k = 0; k < PARTICLE_COUNT; ++k) {
			collisions[i][k] = 0;
		}
	}

	//Collisions are calculated here and adjusts both particles.
	for (int i = 0; i < PARTICLE_COUNT; ++i) {
		for (int k = 0; k < PARTICLE_COUNT; ++k) {
			if (i != k && collisions[i][k] != true && collisions[k][i] != true) {
				double r1 = particles[i].getRadius();
				double x1 = nextPosition[i].getX();
				double y1 = nextPosition[i].getY();

				double r2 = particles[k].getRadius();
				double x2 = nextPosition[k].getX();
				double y2 = nextPosition[k].getY();
				
				double dist = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
				//Contact is true if their distance is the sum
				//of the particles' radiuses or within each other.
				if ( (dist - (r1 + r2)) <= 0) {

					//Mass, velocity, and angle are taken.
					double m1 = particles[i].getMass();
					double v1x = nextVelocity[i].getX();
					double v1y = nextVelocity[i].getY();
					double vNet1 = sqrt(v1x * v1x + v1y * v1y);
					double xi1 = particles[i].getPos().getX();
					double yi1 = particles[i].getPos().getY();
					double ang1 = atan2((y1 - yi1), (x1 - xi1));

					double m2 = particles[k].getMass();
					double v2x = nextVelocity[k].getX();
					double v2y = nextVelocity[k].getY();
					double vNet2 = sqrt(v2x * v2x + v2y * v2y);
					double xi2 = particles[k].getPos().getX();
					double yi2 = particles[k].getPos().getY();
					double ang2 = atan2((y2 - yi2), (x2 - xi2));

					//Collision components of new Velocity
					//are calculated here and applied.
					double phi = atan2(y2 - y1, x2 - x1);
					double eq = (vNet1 * cos(d * (ang1 - phi)) * (m1 - m2) + 2 * m2 * vNet2 * cos(d * (ang2 - phi)) ) / (m1 + m2);
					double v1xf = eq * cos(d * phi) + vNet1 * sin(d * (ang1 - phi)) * cos(d * (phi + M_PI/2));
					double v1yf = eq * sin(d * phi) + vNet1 * sin(d * (ang1 - phi)) * sin(d * (phi + M_PI/2));
					eq = (vNet2 * cos(d * (ang2 - phi)) * (m2 - m1) + 2 * m1 * vNet1 * cos(d * (ang1 - phi)) ) / (m1 + m2);
					double v2xf = eq * cos(d * phi) + vNet2 * sin(d * (ang2 - phi)) * cos(d * (phi + M_PI/2));
					double v2yf = eq * sin(d * phi) + vNet2 * sin(d * (ang2 - phi)) * sin(d * (phi + M_PI/2));

					//Mark the event and do not repeat.
					//Update the changes to both particles.
					collisions[i][k] = true;
					collisions[k][i] = true;
					nextVelocity[i] = Point(v1xf, v1yf);
					nextVelocity[k] = Point(v2xf, v2yf);
					nextPosition[i] = Point(xi1, yi1);
					nextPosition[k] = Point(xi2, yi2);
				}
			}
		}
	}

    //Final calculations: Move particles and update velocities.
    for (int i = 0; i < PARTICLE_COUNT; ++i) { 

	particles[i].setP(nextPosition[i]);
	particles[i].setV(nextVelocity[i].getX() + nextAcceleration[i].getX() * dt, nextVelocity[i].getY() + nextAcceleration[i].getY() * dt);
    }
    // Replace with your game logic!
}

void Game::render()
{
    SDL_RenderClear(renderer);
    
    // rendering here would place objects beneath the particles
    
    for (const Particle& p : particles)
    {
        drawParticle(p);
    }
    
    // rendering here would place objects on top of the particles
    
    SDL_RenderPresent(renderer);
}

void Game::handleEvent(const SDL_Event& event)
{
    switch (event.type)
    {
    // Add your own event handling here if desired
    case SDL_QUIT:
        running = false;
        break;
    default:
        break;
    }
}

void Game::drawParticle(const Particle& p)
{
    SDL_Rect dst;
    double shift = p.getRadius();
    dst.x = (int) (p.getPos().getX() - shift);
    dst.y = (int) (p.getPos().getY() - shift);
    dst.w = shift * 2;
    dst.h = shift * 2;
    SDL_RenderCopy(renderer, particleTexture, NULL, &dst);
}

Particle Game::randomParticle() const
{
    Point pos(rand() % width, rand() % height);
    
    // using some percentage of the mass of Jupiter
    double mass = ((double) rand() / RAND_MAX) * 100;
    
    return Particle(pos, mass);
}
