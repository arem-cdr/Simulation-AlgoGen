#include <math.h>

#include "constantes.h"
#include "robot.h"

#include <iostream>
#include <Windows.h>

using namespace std;


double radian(double angle)
{
    return angle*PI/180;
}

double degre(double angle)
{
    return angle*180/PI;
}

double val_abs(double a)
{
    if(a>0)
        return a;
    return -a;
}

double part_pos(double a)
{
    if(a>0)
        return a;
    return 0;
}

double dist(double a, double b)
{
    return sqrt(a*a + b*b);
}

float properAngleRad(float ang)
{
    if (ang > PI)
        return ang-2*PI;
    if (ang <= -PI)
        return ang+2*PI;
    return ang;
}

int sgn(double a)
{
    if (a>=0)
        return 1;
    return -1;
}

double xconv(int x){ // ces fonctions effectuent les conversions de cm vers l'unite utilisee dans cette simulation
    return (double) (WINDOW_WIDTH-x*WINDOW_WIDTH/3000);
}
double yconv(int y){
    return (double) (y*WINDOW_HEIGHT/2000);
}
double aconv(int angle){
    return (double) (PI-angle*PI/180);
}

robot::robot() : m_pos(xconv(250), yconv(705), aconv(90)), m_target(xconv(330), yconv(1200), aconv(0)), m_posTest(), m_posDep(800, 500, PI/2)
{
    m_length = 110;
    m_width = 100;
    m_rightSpeed = 0.1;
    m_leftSpeed = 0;
    m_maxAcceleration = 0.1;
    m_delay = 2000;
    m_collision = false;
    m_nbMutations = 0;
    m_malus = 0;

    sf::RectangleShape shape(sf::Vector2f(m_length, m_width));
    shape.setOrigin(sf::Vector2f(m_length/2, m_width/2));
    shape.setFillColor(sf::Color::Blue);
    shape.setOutlineColor(sf::Color::Black);
    shape.setOutlineThickness(2);
    shape.setPosition(sf::Vector2f(m_pos.getX(), m_pos.getY()));
    shape.setRotation(-degre(m_pos.getangle()) - 90);

    m_shape = shape;

    shape.setFillColor(sf::Color(20, 100, 20, 170));
    shape.setPosition(sf::Vector2f(m_pos.getX()+100, m_pos.getY()));
    shape.setRotation(-degre(m_pos.getangle()) - 90);

    m_shapeTest = shape;

    shape.setFillColor(sf::Color(0, 0, 255, 150));
    shape.setPosition(sf::Vector2f(m_target.getX(), m_target.getY()));
    shape.setRotation(-degre(m_target.getangle()) - 90);

    m_shapeTarget = shape;

    //gen
    for (int i=0; i<NB_SOL+1; i++)
    {
        for (int j=0; j<NB_TOURS_SIMULES+1; j++)
        {
            m_sol[i][j][0] =  0;
            m_sol[i][j][1] =  0;
        }
    }

    for (int j=0; j<NB_TOURS_SIMULES; j++)
    {
        m_posEn[0][j] =  0;
        m_posEn[1][j] =  0;
    }
}

robot::robot(double x, double y, double angle, sf::Color color) : m_pos(x, y, angle), m_target(400, 100, PI/2), m_posTest(), m_posDep()
{
    m_length = 110;
    m_width = 100;
    m_rightSpeed = 1;
    m_leftSpeed = 2;
    m_maxAcceleration = 0.1;
    m_delay = 2000;
    m_collision = false;
    m_malus = 0;

    sf::RectangleShape shape(sf::Vector2f(m_length, m_width));
    shape.setOrigin(sf::Vector2f(m_length/2, m_width/2));
    shape.setFillColor(color);
    shape.setOutlineColor(sf::Color::Black);
    shape.setOutlineThickness(2);
    shape.setPosition(sf::Vector2f(m_pos.getX(), m_pos.getY()));
    shape.setRotation(-degre(m_pos.getangle()) - 90);

    m_shape = shape;

    shape.setFillColor(sf::Color(255, 0, 0, 150));
    shape.setPosition(sf::Vector2f(m_target.getX(), m_target.getY()));
    shape.setRotation(-degre(m_target.getangle()) - 90);

    m_shapeTarget = shape;

    //gen
    for (int i=0; i<NB_SOL+1; i++)
    {
        for (int j=0; j<NB_TOURS_SIMULES+1; j++)
        {
            m_sol[i][j][0] =  0;
            m_sol[i][j][1] =  0;
        }
    }

    for (int j=0; j<NB_TOURS_SIMULES; j++)
    {
        m_posEn[0][j] =  0;
        m_posEn[1][j] =  0;
    }
}

robot::~robot()
{
    //dtor
}


sf::RectangleShape robot::draw()
{
    return m_shape;
}

sf::RectangleShape robot::drawTest()
{
    return m_shapeTest;
}

sf::RectangleShape robot::drawTarget()
{
    return m_shapeTarget;
}

void robot::update(sf::Time time)
{
    if (m_delay==0)
    {
        frottements(time.asMilliseconds());

        actualise_position(m_rightSpeed, m_leftSpeed);

        m_shape.setPosition(sf::Vector2f(m_pos.getX(), m_pos.getY()));
        m_shape.setRotation(-degre(m_pos.getangle()) - 90);
    }
    else
    {
        m_delay -= time.asMilliseconds();
        if (m_delay<0)
            m_delay = 0;
    }
}

void robot::updateClavier(float vitD, float vitG) // utile au debug : permet de tester le comportement du robot en deplacant la target au clavier
{
	actualise_positionTarget(vitD, vitG);
	//actualise_position(vitD,vitG); // on peut aussi controler le robot au clavier en inversant les lignes commentees dans cette fonction

	m_shapeTarget.setPosition(sf::Vector2f(m_target.getX(), m_target.getY()));
	m_shapeTarget.setRotation(-degre(m_target.getangle()) - 90);
	//m_shape.setPosition(sf::Vector2f(m_pos.getX(), m_pos.getY()));
	//m_shape.setRotation(-degre(m_pos.getangle()) - 90);
}

void robot::frottements(int time)
{
    m_leftSpeed *= 1 - 0.005 * time * FPS / 1000;
    m_rightSpeed *= 1 - 0.005 * time * FPS/ 1000;
}

void robot::actualise_position(float rightSpeed, float leftSpeed)
{
    // determination du cercle décrit par la trajectoire et de la vitesse du robot sur ce cercle

    if (rightSpeed != leftSpeed){

    	double R = 0; // rayon du cercle decrit par la trajectoire
	    double d = 0; // vitesse du robot
        double cx = 0; // position du centre du cercle decrit par la trajectoire
        double cy = 0;

        R = ECART_ROUE / 2 * (rightSpeed + leftSpeed) / (leftSpeed - rightSpeed); // rayon du cercle
        cx = m_pos.getX() + R * sin(m_pos.getangle());
        cy = m_pos.getY() + R * cos(m_pos.getangle());
        d = (leftSpeed + rightSpeed) * 0.5;

        // mise à jour des coordonnées du robot
        if (leftSpeed + rightSpeed != 0){
            m_pos.Setangle(m_pos.getangle() - d/R);//m_angle -= d/R;
        }
        else{
            m_pos.Setangle(m_pos.getangle() + rightSpeed*2.0 / ECART_ROUE); //m_angle += rightSpeed*2.0 / ECART_ROUE;
        }

        if (m_pos.getangle() > PI)
        {
            m_pos.Setangle(m_pos.getangle() - 2*PI); //m_angle -= 2*PI;
        }
        else if (m_pos.getangle() <= -PI)
        {
            m_pos.Setangle(m_pos.getangle() + 2*PI); //m_angle += 2*PI;
        }

        m_pos.setX(cx - R * sin(m_pos.getangle())); //m_xPos = cx - R * sin(m_angle);
        m_pos.setY(cy - R * cos(m_pos.getangle())); //m_yPos = cy - R * cos(m_angle);
    }
    else if (leftSpeed == rightSpeed){ // cas où la trajectoire est une parfaite ligne droite

        m_pos.setX(m_pos.getX() + leftSpeed * cos(m_pos.getangle())); //m_xPos += leftSpeed * cos(m_angle);
        m_pos.setY(m_pos.getY() - rightSpeed * sin(m_pos.getangle())); //m_yPos -= rightSpeed * sin(m_angle);
    }
}

void robot::actualise_positionTarget(float rightSpeed, float leftSpeed)
{
	// determination du cercle décrit par la trajectoire et de la vitesse du robot sur ce cercle

	if (rightSpeed != leftSpeed) {

		double R = 0; // rayon du cercle decrit par la trajectoire
		double d = 0; // vitesse du robot
		double cx = 0; // position du centre du cercle decrit par la trajectoire
		double cy = 0;

		R = ECART_ROUE / 2 * (rightSpeed + leftSpeed) / (leftSpeed - rightSpeed); // rayon du cercle
		cx = m_target.getX() + R * sin(m_target.getangle());
		cy = m_target.getY() + R * cos(m_target.getangle());
		d = (leftSpeed + rightSpeed) * 0.5;

		// mise à jour des coordonnées du robot
		if (leftSpeed + rightSpeed != 0) {
			m_target.Setangle(m_target.getangle() - d / R);//m_angle -= d/R;
		}
		else {
			m_target.Setangle(m_target.getangle() + rightSpeed * 2.0 / ECART_ROUE); //m_angle += rightSpeed*2.0 / ECART_ROUE;
		}

		if (m_target.getangle() > PI)
		{
			m_target.Setangle(m_target.getangle() - 2 * PI); //m_angle -= 2*PI;
		}
		else if (m_target.getangle() <= -PI)
		{
			m_target.Setangle(m_target.getangle() + 2 * PI); //m_angle += 2*PI;
		}

		m_target.setX(cx - R * sin(m_target.getangle())); //m_xPos = cx - R * sin(m_angle);
		m_target.setY(cy - R * cos(m_target.getangle())); //m_yPos = cy - R * cos(m_angle);
	}
	else if (leftSpeed == rightSpeed) { // cas où la trajectoire est une parfaite ligne droite

		m_target.setX(m_target.getX() + leftSpeed * cos(m_target.getangle())); //m_xPos += leftSpeed * cos(m_angle);
		m_target.setY(m_target.getY() - rightSpeed * sin(m_target.getangle())); //m_yPos -= rightSpeed * sin(m_angle);
	}
}

bool robot::delay()
{
    if (m_delay>0)
        return true;
    return false;
}

//genetique

void robot::startGen(double x, double y) // appelée dans le main, setup tout avant de lancer la recherche de trajectoires
{
    int xprec = m_posEn[0][0];
    int yprec = m_posEn[1][0];

    //estimation de la pos ennemie
    m_posEn[0][0] = x;
    m_posEn[1][0] = y;

    for (int j=1; j<NB_TOURS_SIMULES; j++)
    {
		m_posEn[0][j] = m_posEn[0][0] + (m_posEn[0][0] - xprec) * j;
		m_posEn[1][j] = m_posEn[1][0] + (m_posEn[1][0] - yprec) * j;
    }

    m_posTest.Setall(m_pos.getX(), m_pos.getY(), m_pos.getangle());
    if (reachTarget())
    {
        m_delay = 200;
        m_rightSpeed = 0;
        m_leftSpeed = 0;
        retarget();
    }

    //initialise les variables, prepare aux mutations

    m_posTest.Setall(m_pos.getX(), m_pos.getY(), m_pos.getangle());
    actualisePositionTest(m_rightSpeed* TEMPS_GEN * 60 / 1000, m_leftSpeed* TEMPS_GEN * 60 / 1000, -1);
    m_posDep.Setall(m_posTest.getX(), m_posTest.getY(), m_posTest.getangle());

    //remise a 0 des scores des sol
	for (int i = 0; i < NB_SOL; i++)
	{
		m_sol[i][NB_TOURS_SIMULES][0] = 0;
		m_sol[i][NB_TOURS_SIMULES][1] = 0;
	}
    
    m_nbMutations = 0;

    //creation de la population de depart

    //decalage de la meilleure ancienne sol d'une unite de temps
    for (int j=0; j<NB_TOURS_SIMULES-1; j++)
    {
        m_sol[0][j][0] =  m_sol[0][j+1][0];
        m_sol[0][j][1] =  m_sol[0][j+1][1];
    }
    m_sol[0][NB_TOURS_SIMULES][0] = 0;
    m_sol[0][NB_TOURS_SIMULES][1] = 0;


    for (int i=0; i<NB_TOURS_SIMULES; i++) //tourne sur place a droite
    {
        m_sol[1][i][0] =  -1;
        m_sol[1][i][1] =  1;
    }

    for (int i=0; i<NB_TOURS_SIMULES; i++) //reste sur place
    {
        m_sol[2][i][0] =  0;
        m_sol[2][i][1] =  0;
    }

    for (int i=0; i<NB_TOURS_SIMULES; i++) //tourne sur place a droite
    {
        m_sol[3][i][0] =  1;
        m_sol[3][i][1] =  -1;
    }

    for (int i=0; i<NB_TOURS_SIMULES; i++) //avance tout droit
    {
        m_sol[4][i][0] =  2;
        m_sol[4][i][1] =  2;
    }

	for (int i = 5; i < NB_SOL; i++) //les autres sol sont mises à 0, on peut ajouter d'autres mecanismes de generation de sol initiales si besoin
	{
		for (int j= 0; j<NB_TOURS_SIMULES+1; j++) //avance tout droit
		{
			m_sol[i][j][0] = 0;
			m_sol[i][j][1] = 0;
		}
	}

    //rend les sol generees conformes
    for (int j = 0; j<NB_SOL; j++)
    {
        if (m_sol[j][0][0] > m_rightSpeed+0.5)
            m_sol[j][0][0] = m_rightSpeed+0.5;
        if (m_sol[j][0][0] < m_rightSpeed-1)
            m_sol[j][0][0] = m_rightSpeed-1;
        if (m_sol[j][0][1] > m_leftSpeed+0.5)
            m_sol[j][0][1] = m_leftSpeed+0.5;
        if (m_sol[j][0][1] < m_leftSpeed-1)
            m_sol[j][0][1] = m_leftSpeed-1;

        for (int i=1; i<NB_TOURS_SIMULES; i++)
        {
            if (m_sol[j][i][0] > m_sol[j][i-1][0]+0.5)
                m_sol[j][i][0] = m_sol[j][i-1][0]+0.5;
            if (m_sol[j][i][0] < m_sol[j][i-1][0]-1)
                m_sol[j][i][0] = m_sol[j][i-1][0]-1;
            if (m_sol[j][i][1] > m_sol[j][i-1][1]+0.5)
                m_sol[j][i][1] = m_sol[j][i-1][1]+0.5;
            if (m_sol[j][i][1] < m_sol[j][i-1][1]-1)
                m_sol[j][i][1] = m_sol[j][i-1][1]-1;
            if (m_sol[j][i][0] > 5)
                m_sol[j][i][0] = 5;
            if (m_sol[j][i][1] > 5)
                m_sol[j][i][1] = 5;
        }
    }

    //teste les solutions de depart et les classe

    //simulation des sol de depart
    for (int j=0; j<NB_SOL; j++)
    {
        m_malus = 0;
        m_collision = false;

        m_posTest.Setall(m_posDep.getX(), m_posDep.getY(), m_posDep.getangle());

        //simulation des tours

        for (int i=0; i<NB_TOURS_SIMULES; i++)
        {
            actualisePositionTest(m_sol[j][i][0]* TEMPS_GEN * 60 / 1000, m_sol[j][i][1]* TEMPS_GEN * 60 / 1000, i);
            m_sol[j][NB_TOURS_SIMULES][0] += evalueSol()/NB_TOURS_SIMULES; // on fait la moyenne du score de toutes les positions par lesquelle est passe le robot
        }

        if (evalueSol() < 900) // si le robot est encore loin de la target, le score de la solution est finalement seulement la position finale
            m_sol[j][NB_TOURS_SIMULES][0] = evalueSol()-500;

        distSolTest(j);
    }

    //triage des sol
    for (int i=0; i<NB_SOL; i++)
        swapSol(chercheMeilleure(i), i);
}

void robot::endGen() // appelée à la fin de la recherche génétique pour mettre à jour les vitesses appliquees au robot
{
	m_posTest.Setall(m_posDep.getX(), m_posDep.getY(), m_posDep.getangle());

	for (int i = 0; i < NB_TOURS_SIMULES; i++) // affichage du robot vert
	{
		actualisePositionTest(m_sol[0][i][0] * TEMPS_GEN / 17, m_sol[0][i][1] * TEMPS_GEN / 17, i);
	}

	m_shapeTest.setPosition(m_posTest.getX(), m_posTest.getY());
	m_shapeTest.setRotation(-degre(m_posTest.getangle()) - 90);

	// mise à jour des vitesse des roues du robot

	m_collision = false;
	m_posTest.Setall(m_posDep.getX(), m_posDep.getY(), m_posDep.getangle());

	actualisePositionTest(m_sol[0][0][0] * TEMPS_GEN / 17, m_sol[0][0][1] * TEMPS_GEN / 17, 0);

	if (!m_collision && m_delay == 0)
	{
		m_rightSpeed = m_sol[0][0][0];
		m_leftSpeed = m_sol[0][0][1];
	}
	else
	{
		m_rightSpeed = 0;
		m_leftSpeed = 0;
	}
}

void robot::swapSol(int sol1, int sol2)
{
    //echange les positions de sol1 et sol2. Optimisable en le faisant avec des pointeurs. Cette optimisation est d'autant pluys nécessaire que NB_SOL est grand
    if (sol1 != sol2)
    {
        //copie de m_sol[sol1] dans un tableau temporaire
        double solTemp[NB_TOURS_SIMULES+1][2];
        for (int i=0; i<NB_TOURS_SIMULES+1; i++)
        {
            solTemp[i][0]=m_sol[sol1][i][0];
            solTemp[i][1]=m_sol[sol1][i][1];
        }

        //copie de m_sol[sol2] dans m_sol[sol1]
        for (int i=0; i<NB_TOURS_SIMULES+1; i++)
        {
            m_sol[sol1][i][0]=m_sol[sol2][i][0];
            m_sol[sol1][i][1]=m_sol[sol2][i][1];
        }

        //copie du tableau temporaire dans m_sol[sol2]
        for (int i=0; i<NB_TOURS_SIMULES+1; i++)
        {
            m_sol[sol2][i][0]=solTemp[i][0];
            m_sol[sol2][i][1]=solTemp[i][1];
        }
    }
}

int robot::chercheMeilleure(int indiceDepart) //const
{
    //cherche la meilleure sol parmi celles apres l'indice de depart dans m_sol

    int a = indiceDepart;
    for (int i=indiceDepart+1; i<NB_SOL; i++)
    {
        if (m_sol[a][NB_TOURS_SIMULES][0]<m_sol[i][NB_TOURS_SIMULES][0])
            a = i;
    }
    return a;
}

double robot::evalueSol() const // fonction cruciale : c'est grace a la note retournee par cette fonction que l'on va evaluer les performances d'une solution
{
    float d = dist(m_target.getX()-m_posTest.getX(), m_target.getY()-m_posTest.getY()); // la distance entre la pos actuelle du robot et sa target
    float a = 512;
    if (d<5) // on considere a seulement si le robot est suffisamment proche de la cible
    {
		a = val_abs(properAngleRad(m_target.getangle() - m_posTest.getangle())) * 160; // la difference d'angle entre la pos actuelle du robot et sa target
    } 
    return 1000 - d - a - m_malus; //la note max est de 1000. m_malus augmente en fonction des obstacles rencontres par le robot
}

void robot::genSol()
{
	//genere une nouvelle solution et la teste

	m_collision = false;

	double a = (double)rand() / RAND_MAX * 10.0;
	int ind1 = floor(a * a / 100 * NB_SOL); // genere un entier entre 0 et NB_SOL-1 inclus, avec une proba plus grande pour les petits entiers
	a = (double)rand() / RAND_MAX * 10.0;
	int ind2 = floor(a * a / 100 * NB_SOL);
	int indCr = floor(rand() / RAND_MAX * (NB_TOURS_SIMULES - 1) + 1);

	crossOver(ind1, ind2, indCr);
	mutate();
	while ((float)rand() / RAND_MAX < 0.3)
		mutate();

	m_nbMutations++;

	//teste si la sol generee est conforme
	if (m_sol[NB_SOL][0][0] > m_rightSpeed + 0.5)
		m_sol[NB_SOL][0][0] = m_rightSpeed + 0.5;
	if (m_sol[NB_SOL][0][0] < m_rightSpeed - 0.5)
		m_sol[NB_SOL][0][0] = m_rightSpeed - 0.5;
	if (m_sol[NB_SOL][0][1] > m_leftSpeed + 0.5)
		m_sol[NB_SOL][0][1] = m_leftSpeed + 0.5;
	if (m_sol[NB_SOL][0][1] < m_leftSpeed - 0.5)
		m_sol[NB_SOL][0][1] = m_leftSpeed - 0.5;
	if (m_sol[NB_SOL][0][0] > 5)
		m_sol[NB_SOL][0][0] = 5;
	if (m_sol[NB_SOL][0][1] > 5)
		m_sol[NB_SOL][0][1] = 5;

	for (int i = 1; i < NB_TOURS_SIMULES; i++)
	{
		if (m_sol[NB_SOL][i][0] > m_sol[NB_SOL][i - 1][0] + 0.5)
			m_sol[NB_SOL][i][0] = m_sol[NB_SOL][i - 1][0] + 0.5;
		if (m_sol[NB_SOL][i][0] < m_sol[NB_SOL][i - 1][0] - 0.5)
			m_sol[NB_SOL][i][0] = m_sol[NB_SOL][i - 1][0] - 0.5;
		if (m_sol[NB_SOL][i][1] > m_sol[NB_SOL][i - 1][1] + 0.5)
			m_sol[NB_SOL][i][1] = m_sol[NB_SOL][i - 1][1] + 0.5;
		if (m_sol[NB_SOL][i][1] < m_sol[NB_SOL][i - 1][1] - 0.5)
			m_sol[NB_SOL][i][1] = m_sol[NB_SOL][i - 1][1] - 0.5;
		if (m_sol[NB_SOL][i][0] > 5)
			m_sol[NB_SOL][i][0] = 5;
		if (m_sol[NB_SOL][i][1] > 5)
			m_sol[NB_SOL][i][1] = 5;
	}

	//teste la solution NB_SOL et la classe parmi les autres solutions a partir de m_sol deja classe
	m_malus = 0;
	m_collision = false;

	m_posTest.Setall(m_posDep.getX(), m_posDep.getY(), m_posDep.getangle());

	//simulation des tours
	m_sol[NB_SOL][NB_TOURS_SIMULES][0] = 0;

	for (int i = 0; i < NB_TOURS_SIMULES; i++)
	{
		actualisePositionTest(m_sol[NB_SOL][i][0] * TEMPS_GEN*60/1000, m_sol[NB_SOL][i][1] * TEMPS_GEN*60/1000, i);
		m_sol[NB_SOL][NB_TOURS_SIMULES][0] += evalueSol() / NB_TOURS_SIMULES;
		if (m_collision) // si une collision s'est produite, on ne prend pas en compte la solution generee
		{
			return;
		}
	}

	if (evalueSol() < 400)
		m_sol[NB_SOL][NB_TOURS_SIMULES][0] = evalueSol() - 500;

	distSolTest(NB_SOL);

	//on place la sol trouvee parmi les sol existantes par une recherche dicotomique
	int classement = NB_SOL;
	int dico1 = 0;
	int dico2 = NB_SOL;

	if (m_sol[NB_SOL][NB_TOURS_SIMULES][0] > m_sol[NB_SOL - 1][NB_TOURS_SIMULES][0]) // petite optimisation : souvent la sol generee sera trop mauvaise pour etre gardee
	{
		while (dico2 - dico1 > 1)
		{
			if (m_sol[NB_SOL][NB_TOURS_SIMULES][0] > m_sol[(int) ((dico1 + dico2) * 0.5)][NB_TOURS_SIMULES][0])
			{
				dico2 = (int) ((dico1 + dico2) * 0.5);
			}
			else
			{
				dico1 = (int) ((dico1 + dico2) * 0.5);
			}
		}
		if (m_sol[NB_SOL][NB_TOURS_SIMULES][0] > m_sol[dico1][NB_TOURS_SIMULES][0])
		{
			classement = dico1;
		}
		else
		{
			classement = dico2;
		}
	}

    //verification de la distance de la sol generee par rapport aux autres sol. /!\ PAS FONCTIONNEL
    int d = NB_SOL; //vaut NB_SOL si aucune des autre sol n'est proche de celle generee
    if (classement<NB_SOL)
    {
        d = testDist();
    }

    //on reconstitue alors le tableau des sol classe avec la nouvelle sol
    while (/*classement<=d && */classement != NB_SOL)
    {
        swapSol(classement, NB_SOL);
        classement++;
    }
}

void robot::crossOver(int ind1, int ind2, int a)
{
    //mixe les sol ind1 et ind2 pour en creer une nouvelle

    for (int i=0; i<a; i++)
    {
        m_sol[NB_SOL][i][0] = m_sol[ind1][i][0];
        m_sol[NB_SOL][i][1] = m_sol[ind1][i][1];
    }

    for (int i=a; i<NB_TOURS_SIMULES; i++)
    {
        m_sol[NB_SOL][i][0] = m_sol[ind2][i][0];
        m_sol[NB_SOL][i][1] = m_sol[ind2][i][1];
    }
}

void robot::mutate()
{
    float r = (float) rand()/RAND_MAX;
    int a = (int) exp((float) rand()*log(NB_TOURS_SIMULES + 1)/RAND_MAX) -1; //genere un entier entre 0 et NB_TOURS_SIMULES

    if (r<0.25)
        brake(a);
    else if (r<0.5)
        accel(a);
    else if (r<0.75)
        turnLeft(a);
    else if (r<1) // inutile dans ce cas
        turnRight(a);
}

void robot::brake(int ind)
{
    for (int i=ind; i<NB_TOURS_SIMULES; i++)
    {
        m_sol[NB_SOL][i][0] -= 1*(1 - (double) m_nbMutations/35000);
        m_sol[NB_SOL][i][1] -= 1*(1 - (double) m_nbMutations/35000);
    }
}

void robot::accel(int ind)
{
    for (int i=ind; i<NB_TOURS_SIMULES; i++)
    {
        m_sol[NB_SOL][i][0] += 1*(1 - (double) m_nbMutations/35000);
        m_sol[NB_SOL][i][1] += 1*(1 - (double) m_nbMutations/35000);
    }
}

void robot::turnLeft(int ind)
{
    for (int i=ind; i<NB_TOURS_SIMULES; i++)
    {
        m_sol[NB_SOL][i][0] += 1*(1 - (double) m_nbMutations/35000);
        m_sol[NB_SOL][i][1] -= 1*(1 - (double) m_nbMutations/35000);
    }
}

void robot::turnRight(int ind)
{
    for (int i=ind; i<NB_TOURS_SIMULES; i++)
    {
        m_sol[NB_SOL][i][0] -= 1*(1 - (double) m_nbMutations/35000);
        m_sol[NB_SOL][i][1] += 1*(1 - (double) m_nbMutations/35000);
    }
}

void robot::retarget()
{
    double x = (double) (rand()) * 1150 / RAND_MAX + 200;
    double y = (double) (rand()) * 400 / RAND_MAX + 200;
    double ang = ((double) (rand()) / RAND_MAX - 0.5) * PI;

    m_target.Setall(x, y, ang);

    m_shapeTarget.setPosition(sf::Vector2f(m_target.getX(), m_target.getY()));
    m_shapeTarget.setRotation(-degre(m_target.getangle()) - 90);
}


void robot::actualisePositionTest(float rightSpeed, float leftSpeed, int i)
{
    // determination du cercle décrit par la trajectoire et de la vitesse du robot sur ce cercle
    if (rightSpeed != leftSpeed){

    	double R = 0; // rayon du cercle decrit par la trajectoire
	    double d = 0; // vitesse du robot
        double cx = 0; // position du centre du cercle decrit par la trajectoire
        double cy = 0;

        R = ECART_ROUE / 2 * (rightSpeed + leftSpeed) / (leftSpeed - rightSpeed); // rayon du cercle
        cx = m_posTest.getX() + R * sin(m_posTest.getangle());
        cy = m_posTest.getY() + R * cos(m_posTest.getangle());
        d = (leftSpeed + rightSpeed) * 0.5;

        // mise à jour des coordonnées du robot
        if (leftSpeed + rightSpeed != 0){
            m_posTest.Setangle(m_posTest.getangle() - d/R);//m_angle -= d/R;
        }
        else{
            m_posTest.Setangle(m_posTest.getangle() + rightSpeed*2.0 / ECART_ROUE); //m_angle += rightSpeed*2.0 / ECART_ROUE;
        }

        if (m_posTest.getangle() > PI)
        {
            m_posTest.Setangle(m_posTest.getangle() - 2*PI); //m_angle -= 2*PI;
        }
        else if (m_posTest.getangle() <= -PI)
        {
            m_posTest.Setangle(m_posTest.getangle() + 2*PI); //m_sangle += 2*PI;
        }

        m_posTest.setX(cx - R * sin(m_posTest.getangle())); //m_xPos = cx - R * sin(m_angle);
        m_posTest.setY(cy - R * cos(m_posTest.getangle())); //m_yPos = cy - R * cos(m_angle);
    }
    else if (leftSpeed == rightSpeed){ // cas où la trajectoire est une parfaite ligne droite
        m_posTest.setX(m_posTest.getX() + leftSpeed * cos(m_posTest.getangle())); //m_xPos += leftSpeed * cos(m_angle);
        m_posTest.setY(m_posTest.getY() - rightSpeed * sin(m_posTest.getangle())); //m_yPos -= rightSpeed * sin(m_angle);
    }

    testCollisionTest();

    if (i>=0) // si i < 0 : ???
        testCollisionEn(i);
}

void robot::testCollisionEn(int i)
{
    double d = dist(m_posTest.getX() - m_posEn[0][i], m_posTest.getY() - m_posEn[1][i]);
    double v = val_abs(m_sol[NB_SOL][i][0]+m_sol[NB_SOL][i][0]);
    if (d < 130)//min(50*(i+1), 200))
    {
        m_malus += (200 - d) + 500;
        m_collision = true;
    }
    else if(d<min(70*(i+1), 250))
    {
        m_malus += ((250 - d))*max(v-2,0.1);
    }
}

void robot::testCollisionTest()
{
    if(m_posTest.getX()<m_length)
    {
        //cout << "collision1" << endl;
        m_malus+=(m_length-m_posTest.getX())/20;
        testLeftCollisionTest();
    }
    else if(m_posTest.getX()>WINDOW_WIDTH-m_length)
    {
        m_malus+=(m_posTest.getX()-WINDOW_WIDTH + m_length)/20;
        testRightCollisionTest();
    }
    if(m_posTest.getY()<m_length)
    {
        //cout << "collision3" << endl;
        m_malus+=(m_length-m_posTest.getY())/20;
        testUpCollisionTest();
    }
    else if(m_posTest.getY()>WINDOW_HEIGHT-m_length - 220)
    {
        //cout << "collision4" << endl;
        m_malus+=(m_posTest.getY()-WINDOW_HEIGHT + m_length + 220)/20;
        testBotCollisionTest();
    }
    if (dist(m_posTest.getX() - WINDOW_WIDTH/2, m_posTest.getY() - (WINDOW_HEIGHT - 300)) < sqrt(m_width*m_width + m_length*m_length)/1.8) //collision avec le bout de la balance
    {
        m_malus += 200;
        m_collision = true;
    }

}

void robot::testUpCollisionTest()
{
    double diag = sqrt(m_width*m_width + m_length*m_length)/2;

    // 1 : coin en bas à droite

    double angleCar = atan((double) (m_length)/m_width);
    double angle = m_posTest.getangle() - angleCar;

    //double x1 = m_pos.getX() + diag*cos(angle);
    double y1 = m_posTest.getY() - diag*sin(angle);

    if(y1<0)
    {
        m_collision = true;
        m_malus += -y1*2 + 200;
    }

    // 2 : coin en haut à droite

    angle = m_posTest.getangle() + angleCar;

    //x1 = m_pos.getX() + diag*cos(angle);
    y1 = m_posTest.getY() - diag*sin(angle);

    if(y1<0)
    {
        m_collision = true;
        m_malus += -y1*2 + 200;
    }

    // 3 : coin en haut à gauche

    angle = m_posTest.getangle() - PI - angleCar;
    //x1 = m_pos.getX() + diag*cos(angle);:
    y1 = m_posTest.getY() - diag*sin(angle);

    if(y1<0)
    {
        m_collision = true;
        m_malus += -y1*2 + 200;
    }

    // 4 : coin en bas à gauche

    angle = m_posTest.getangle() - PI + angleCar;

    //x1 = m_pos.getX() + diag*cos(angle);
    y1 = m_posTest.getY() - diag*sin(angle);

    if(y1<0)
    {
        m_collision = true;
        m_malus += -y1*2 + 200;
    }
}

void robot::testBotCollisionTest()
{
    /*
    s'il y a collision, calcule la distance dépassée "depassement", puis reactualise la position avec ce depassement.
    */

    double diag = sqrt(m_width*m_width + m_length*m_length)/2;

    // 1 : coin en bas à droite

    double angleCar = atan((double) (m_length)/m_width);
    double angle = m_posTest.getangle() - angleCar;

    //double x1 = m_pos.getX() + diag*cos(angle);
    double y1 = m_posTest.getY() - diag*sin(angle);

    if(y1>WINDOW_HEIGHT - 220)
    {
        m_collision = true;
        m_malus += (y1 - (WINDOW_HEIGHT - 220))*2 + 200;
    }

    // 2 : coin en haut à droite

    angle = m_posTest.getangle() + angleCar;

    //x1 = m_pos.getX() + diag*cos(angle);
    y1 = m_posTest.getY() - diag*sin(angle);

    if(y1>WINDOW_HEIGHT - 220)
    {
        m_collision = true;
        m_malus += (y1 - (WINDOW_HEIGHT - 220))*2 + 200;
    }

    // 3 : coin en haut à gauche

    angle = m_posTest.getangle() - PI - angleCar;
    //x1 = m_pos.getX() + diag*cos(angle);:
    y1 = m_posTest.getY() - diag*sin(angle);

    if(y1>WINDOW_HEIGHT - 220)
    {
        m_collision = true;
        m_malus += (y1 - (WINDOW_HEIGHT - 220))*2 + 200;
    }

    // 4 : coin en bas à gauche

    angle = m_posTest.getangle() - PI + angleCar;

    //x1 = m_pos.getX() + diag*cos(angle);
    y1 = m_posTest.getY() - diag*sin(angle);

    if(y1>WINDOW_HEIGHT - 220)
    {
        m_collision = true;
        m_malus += (y1 - (WINDOW_HEIGHT - 220))*2 + 200;
    }
}

void robot::testLeftCollisionTest()
{
    double diag = sqrt(m_width*m_width + m_length*m_length)/2;

    // 1 : coin en bas à droite

    double angleCar = atan((double) (m_length)/m_width);
    double angle = m_posTest.getangle() - angleCar;

    double x1 = m_posTest.getX() + diag*cos(angle);
    //double y1 = m_posTest.getY() - diag*sin(angle);

    if(x1<0)
    {
        m_collision = true;
        m_malus += -x1*2 + 200;
    }

    // 2 : coin en haut à droite

    angle = m_posTest.getangle() + angleCar;

    x1 = m_posTest.getX() + diag*cos(angle);
    //y1 = m_posTest.getY() - diag*sin(angle);

    if(x1<0)
    {
        m_collision = true;
        m_malus += -x1*2 + 200;
    }

    // 3 : coin en haut à gauche

    angle = m_posTest.getangle() - PI - angleCar;

    x1 = m_posTest.getX() + diag*cos(angle);
    //y1 = m_posTest.getY() - diag*sin(angle);

    if(x1<0)
    {
        m_collision = true;
        m_malus += -x1*2 + 200;
    }

    // 4 : coin en bas à gauche

    angle = m_posTest.getangle() - PI + angleCar;

    x1 = m_posTest.getX() + diag*cos(angle);
    //y1 = m_posTest.getY() - diag*sin(angle);

    if(x1<0)
    {
        m_collision = true;
        m_malus += -x1*2 + 200;
    }
}

void robot::testRightCollisionTest()
{
    double diag = sqrt(m_width*m_width + m_length*m_length)/2;

    // 1 : coin en bas à droite

    double angleCar = atan(1.1);//atan((double) (m_length)/m_width);
    double angle = m_posTest.getangle() - angleCar;

    double x1 = m_posTest.getX() + diag*cos(angle);
    //double y1 = m_posTest.getY() - diag*sin(angle);

    if(x1>WINDOW_WIDTH)
    {
        m_collision = true;
        m_malus += (x1 - WINDOW_WIDTH)*2 + 200;
    }

    // 2 : coin en haut à droite

    angle = m_posTest.getangle() + angleCar;

    x1 = m_posTest.getX() + diag*cos(angle);
    //y1 = m_posTest.getY() - diag*sin(angle);

    if(x1>WINDOW_WIDTH)
    {
        m_collision = true;
        m_malus += (x1 - WINDOW_WIDTH)*2 + 200;
    }

    // 3 : coin en haut à gauche

    angle = m_posTest.getangle() - PI - angleCar;

    x1 = m_posTest.getX() + diag*cos(angle);
    //y1 = m_posTest.getY() - diag*sin(angle);

    if(x1>WINDOW_WIDTH)
    {
        m_collision = true;
        m_malus += (x1 - WINDOW_WIDTH)*2 + 200;
    }

    // 4 : coin en bas à gauche

    angle = m_posTest.getangle() - PI + angleCar;

    x1 = m_posTest.getX() + diag*cos(angle);
    //y1 = m_posTest.getY() - diag*sin(angle);

    if(x1>WINDOW_WIDTH)
    {
        m_collision = true;
        m_malus += (x1 - WINDOW_WIDTH)*2 + 200;
    }
}

double robot::getX(){
    return m_pos.getX();
}

double robot::getY(){
    return m_pos.getY();
}

void robot::score(int indSol)
{
    m_sol[indSol][NB_TOURS_SIMULES][0] = (m_sol[indSol][NB_TOURS_SIMULES][0]*0+m_sol[indSol][NB_TOURS_SIMULES][1]*2)/2;
}

void robot::distSolTest(int indSol) 
{
    m_sol[indSol][NB_TOURS_SIMULES][1] = 0;

    for (int i=0; i<NB_TOURS_SIMULES; i++)
    {
        m_sol[indSol][NB_TOURS_SIMULES][1] += val_abs(m_sol[indSol][i][0]-m_sol[indSol][i][1]);
    }

    m_sol[indSol][NB_TOURS_SIMULES][1] *= sgn(m_sol[indSol][0][0]-m_sol[indSol][0][1]+m_sol[indSol][1][0]-m_sol[indSol][1][1]+m_sol[indSol][2][0]-m_sol[indSol][2][1]);
}

int robot::testDist()
{
    for (int i=0; i<NB_TOURS_SIMULES; i++)
    {
        if (val_abs(m_sol[i][NB_TOURS_SIMULES][1] - m_sol[NB_SOL][NB_TOURS_SIMULES][1]) < 4)
        {
            return i;
        }
    }
    return -1;
}

bool robot::reachTarget()
{
    if (dist(m_target.getX()-m_pos.getX(), m_target.getY()-m_pos.getY())<13 && val_abs(properAngleRad(m_target.getangle()-m_pos.getangle()))<0.05 && val_abs(m_rightSpeed) < 0.1 && val_abs(m_leftSpeed) < 0.1)
        return true;
    return false;
}

bool robot::reachTargetTest(int i)
{
    if ((dist(m_target.getX()-m_posTest.getX(), m_target.getY()-m_posTest.getY())<7 && val_abs(properAngleRad(m_target.getangle()-m_posTest.getangle()))<0.03 && val_abs(m_sol[NB_SOL][i][0]) < 0.3 && val_abs(m_sol[NB_SOL][i][0]) < 0.3) && m_collision == false)
    {
        return true;
    }
    return false;
}
