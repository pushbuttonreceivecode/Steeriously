#include <iostream>

#include <SFML/Graphics.hpp>
#include <steeriously/libinc.hpp>

const int WINDOWX = 800;
const int WINDOWY = 600;
const double WALLPADDING = 20.0;
const int OBSTACLES = 50;
const int MINRADIUS = 10.f;
const int MAXRADIUS = 15.f;
const int MAXITERS = 2000;
const int MINGAP = 4.f;
const int MINBORDER = 50.f;
const int flockNum = 100;
std::vector<steer::Wall> m_walls;
std::vector<steer::SphereObstacle> m_obstacles;

void createWalls();
void createObstacles();

int main()
{
    srand(NULL);

    sf::RenderWindow window(sf::VideoMode(WINDOWX,WINDOWY,32), "test", sf::Style::Default);
    sf::Event e;

    //set up walls
    createWalls();

    //set up obstacles
    createObstacles();

    //set up a path
    std::list<steer::Vector2> pathPoints;
    pathPoints.push_back(steer::Vector2(400.0, 100.0));
    pathPoints.push_back(steer::Vector2(200.0, 200.0));
    pathPoints.push_back(steer::Vector2(150.0, 250.0));
    pathPoints.push_back(steer::Vector2(450.0, 300.0));
    pathPoints.push_back(steer::Vector2(400.0, 100.0));
    steer::Path path(5,pathPoints);
    path.loopOn();

    //a flock!!!
    steer::BehaviorParameters fparams;
    fparams.NumAgents = flockNum/2;
    fparams.SeparationWeight = 10.f;
    fparams.AlignmentWeight = 10.f;
    fparams.CohesionWeight = 10.f;
    fparams.SeekWeight = 2.f;
    fparams.WallAvoidanceWeight = 50.f;
    fparams.ObstacleAvoidanceWeight = 50.f;
    fparams.radius = 4.f;
    fparams.MaxForce = 500.f;
    fparams.MaxSpeed = 300.f;
    fparams.DecelerationTweaker = 100.f;
    fparams.MinDetectionBoxLength = 100.f;
    fparams.WallDetectionFeelerLength = 100.f;
    std::vector<steer::SuperComponent> flock;
    std::vector<steer::SuperComponent*> pf;
    std::vector<steer::SuperComponent*> pf2;
    std::vector<sf::RectangleShape> frects;
    steer::Path tpath(5,pathPoints);
    tpath.loopOn();
    steer::BehaviorParameters pparams;
    pparams.radius = 8.f;
    pparams.FollowPathWeight = 5.f;
    pparams.SeekWeight = 1.f;
    pparams.NumAgents = flockNum/2;
    pparams.SeparationWeight = 1.f;
    pparams.AlignmentWeight = 10.f;
    pparams.CohesionWeight = 10.f;
    pparams.MaxForce = 300.f;
    pparams.MaxSpeed = 150.f;
    pparams.DecelerationTweaker = 100.f;

    for(int i=0; i<flockNum; ++i)
    {
        int tx = rand() % 700;
        int ty = rand() % 500;
        sf::RectangleShape temp;

        if(i % 2 == 0)//make this segment of the flock follow the path...
        {
            steer::SuperComponent tf(&pparams);
            tf.setTarget(steer::Vector2(400.0, 300.0));
            tf.setPosition(steer::Vector2(tx, ty));
            tf.flockingOn();
            tf.pathFollowingOn();
            tf.obstacleAvoidanceOff();
            tf.wallAvoidanceOff();
            tf.seekOn();
            tf.setPath(&tpath);
            flock.push_back(tf);

            temp.setFillColor(sf::Color(255,100,0));
            temp.setSize(sf::Vector2f(pparams.radius, pparams.radius));
            temp.setOrigin(pparams.radius/2.f, pparams.radius/2.f);
        }else{//make this segment of the flock follow the mouse...
            steer::SuperComponent tf(&fparams);
            tf.flockingOn();
            tf.setTarget(steer::Vector2(400.0, 300.0));
            tf.setPosition(steer::Vector2(tx, ty));
            flock.push_back(tf);

            temp.setFillColor(sf::Color(100,255,100));
            temp.setSize(sf::Vector2f(fparams.radius, fparams.radius));
            temp.setOrigin(fparams.radius/2.f, fparams.radius/2.f);
        }
        frects.push_back(temp);
    }

    //split up pointers...
    for(int i=0; i<flockNum; ++i)
    {
        if(i % 2 == 0)
        {
            pf2.push_back(&flock[i]);
        }else{
            pf.push_back(&flock[i]);
        }
    }

    std::vector<steer::Wall*> pw;
    for(auto& i : m_walls)
        pw.push_back(&i);

    std::vector<steer::SphereObstacle*> po;
    for(auto& i : m_obstacles)
        po.push_back(&i);

    //flock needs pointer to
    //container of pointers :D
    //COPIES ARE THE DEVIL!!! \m/
    for(int i=0; i<flockNum; ++i)
    {
        //again...split so that each
        //flock segment associates
        //with their own members
        if(i % 2 == 0)
        {
            flock[i].setNeighbors(&pf2);
            flock[i].setWalls(&pw);
            flock[i].setObstacles(&po);
        }else{
            flock[i].setNeighbors(&pf);
            flock[i].setWalls(&pw);
            flock[i].setObstacles(&po);
        }
    }

    /*
    ///////////////////////////////////////
    ///////////////////////////////////////
    The following are separate stand-alone
    components that exhibit each available
    steering behavior
    ///////////////////////////////////////
    ///////////////////////////////////////
    */

    //seeker!
    steer::BehaviorParameters p;
    p.radius = 10.f;
    steer::SeekComponent s(&p);
    s.setTarget(steer::Vector2(400.0, 300.0));
    sf::RectangleShape rect;
    rect.setFillColor(sf::Color(255,0,0));
    rect.setSize(sf::Vector2f(s.getBoundingRadius(), s.getBoundingRadius()));
    rect.setOrigin(sf::Vector2f(s.getBoundingRadius(), s.getBoundingRadius())/2.f);

    //wanderer!!
    steer::WanderComponent w(&p);
    w.setPosition(steer::Vector2(400.0, 300.0));
    sf::RectangleShape wrect;
    wrect.setFillColor(sf::Color(255,255,255));
    wrect.setSize(sf::Vector2f(w.getBoundingRadius(), w.getBoundingRadius()));
    wrect.setOrigin(sf::Vector2f(w.getBoundingRadius(), w.getBoundingRadius())/2.f);

    //an arriver!
    p.DecelerationTweaker = 2.f;
    steer::ArriveComponent a(&p);
    a.setTarget(steer::Vector2(400.0, 300.0));
    a.setPosition(steer::Vector2(600.0, 500.0));
    sf::RectangleShape arect;
    arect.setFillColor(sf::Color(255,0,255));
    arect.setSize(sf::Vector2f(a.getBoundingRadius(), a.getBoundingRadius()));
    arect.setOrigin(sf::Vector2f(a.getBoundingRadius(), a.getBoundingRadius())/2.f);

    //a pursuer!
    steer::PursuitComponent ps(&p);
    ps.setTarget(steer::Vector2(400.0, 300.0));
    ps.setPosition(steer::Vector2(100.0, 500.0));
    ps.setTargetAgent(&w);
    sf::RectangleShape psrect;
    psrect.setFillColor(sf::Color(100,255,255));
    psrect.setSize(sf::Vector2f(ps.getBoundingRadius(), ps.getBoundingRadius()));
    psrect.setOrigin(sf::Vector2f(ps.getBoundingRadius(), ps.getBoundingRadius())/2.f);

    //a flee-er!
    steer::FleeComponent flee(&p);
    flee.setPosition(steer::Vector2(200.0, 500.0));
    sf::RectangleShape fleerect;
    fleerect.setFillColor(sf::Color(255,255,0));
    fleerect.setSize(sf::Vector2f(flee.getBoundingRadius(), flee.getBoundingRadius()));
    fleerect.setOrigin(sf::Vector2f(flee.getBoundingRadius(), flee.getBoundingRadius())/2.f);

    //an evader!
    p.EvadeWeight = 1.f;
    steer::EvadeComponent ev(&p);
    ev.setPosition(steer::Vector2(200.0, 500.0));
    ev.setTargetAgent(&s);
    sf::RectangleShape evrect;
    evrect.setFillColor(sf::Color(255,0,255));
    evrect.setSize(sf::Vector2f(ev.getBoundingRadius(), ev.getBoundingRadius()));
    evrect.setOrigin(sf::Vector2f(ev.getBoundingRadius(), ev.getBoundingRadius())/2.f);

    //a path follower
    p.FollowPathWeight = 5.f;
    steer::PathFollowingComponent pather(&path, &p);
    sf::RectangleShape pathrect;
    pathrect.setFillColor(sf::Color(255,100,100));
    pathrect.setSize(sf::Vector2f(pather.getBoundingRadius(), pather.getBoundingRadius()));
    pathrect.setOrigin(sf::Vector2f(pathrect.getSize().x, pathrect.getSize().y)/2.f);
    std::vector<sf::Vertex> drawablePath;
    for(auto w : pathPoints)
    {
        drawablePath.push_back(sf::Vertex(sf::Vector2f(w.x,w.y), sf::Color::White));
    }

    //a hider!
    p.HideWeight = 10.f;
    steer::HideComponent hc(&p);
    hc.setPosition(steer::Vector2(300.0, 500.0));
    hc.setTargetAgent(&s);
    hc.setObstacles(&po);
    sf::RectangleShape hcrect;
    hcrect.setFillColor(sf::Color(100,100,255));
    hcrect.setSize(sf::Vector2f(hc.getBoundingRadius(), hc.getBoundingRadius()));
    hcrect.setOrigin(sf::Vector2f(hc.getBoundingRadius(), hc.getBoundingRadius())/2.f);

    //an interposer!
    p.InterposeWeight = 50.f;
    p.MaxForce = 1000.f;
    p.MaxSpeed = 500.f;
    steer::InterposeComponent ic(&p);
    ic.setPosition(steer::Vector2(300.0, 500.0));
    ic.setAgents(&pather, &ev);
    sf::RectangleShape icrect;
    icrect.setFillColor(sf::Color(100,255,100));
    icrect.setSize(sf::Vector2f(ic.getBoundingRadius(), ic.getBoundingRadius()));
    icrect.setOrigin(sf::Vector2f(ic.getBoundingRadius(), ic.getBoundingRadius())/2.f);

    //an offset pursuer!
    p.OffsetPursuitWeight = 10.f;
    steer::OffsetPursuitComponent offc(&p);
    offc.setPosition(steer::Vector2(300.0, 500.0));
    offc.setLeader(&ps);
    sf::RectangleShape offcrect;
    offcrect.setFillColor(sf::Color(200,200,255));
    offcrect.setSize(sf::Vector2f(offc.getBoundingRadius(), offc.getBoundingRadius()));
    offcrect.setOrigin(sf::Vector2f(offc.getBoundingRadius(), offc.getBoundingRadius())/2.f);

    sf::Time m_timePerFrame = sf::seconds(1.f/60.f);
    sf::Clock m_clock;
    sf::Time m_timeElapsed = sf::Time::Zero;

    bool running = true;
    while(running)
    {
        //grab mouse coords...
        steer::Vector2 m = steer::Vector2(sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y);

        while(window.pollEvent(e))
        {
            if(e.type == sf::Event::Closed)
            {
                window.close();
                return 0;
            }
            if(e.type == sf::Event::KeyPressed)
            {
                switch(e.key.code)
                {
                    case sf::Keyboard::Escape:
                    {
                        window.close();
                        return 0;
                    }
                    break;
                }
            }
        }
        m_timeElapsed += m_clock.restart();
        while (m_timeElapsed > m_timePerFrame)
        {
            m_timeElapsed -= m_timePerFrame;
            s.Update(m_timePerFrame.asSeconds());
            s.setTarget(m);
            WrapAround(s.m_agentPosition, WINDOWX, WINDOWY);
            w.Update(m_timePerFrame.asSeconds());
            w.setTarget(m);
            WrapAround(w.m_agentPosition, WINDOWX, WINDOWY);
            for(int i=0; i<flockNum; ++i)
            {
                flock[i].Update(m_timePerFrame.asSeconds());

                if(i % 2 != 0)
                    flock[i].setTarget(m);

                WrapAround(flock[i].m_agentPosition, WINDOWX, WINDOWY);
            }
            a.Update(m_timePerFrame.asSeconds());
            a.setTarget(m);
            WrapAround(a.m_agentPosition, WINDOWX, WINDOWY);
            ps.Update(m_timePerFrame.asSeconds());
            WrapAround(ps.m_agentPosition, WINDOWX, WINDOWY);
            flee.Update(m_timePerFrame.asSeconds());
            flee.setTarget(m);
            WrapAround(flee.m_agentPosition, WINDOWX, WINDOWY);
            ev.Update(m_timePerFrame.asSeconds());
            WrapAround(ev.m_agentPosition, WINDOWX, WINDOWY);
            pather.Update(m_timePerFrame.asSeconds());
            hc.Update(m_timePerFrame.asSeconds());
            WrapAround(hc.m_agentPosition, WINDOWX, WINDOWY);
            ic.Update(m_timePerFrame.asSeconds());
            WrapAround(ic.m_agentPosition, WINDOWX, WINDOWY);
            offc.Update(m_timePerFrame.asSeconds());
            WrapAround(offc.m_agentPosition, WINDOWX, WINDOWY);
        }
        rect.setPosition(sf::Vector2f(s.getPosition().x, s.getPosition().y));
        rect.setRotation(s.getRotation());
        arect.setPosition(sf::Vector2f(a.getPosition().x, a.getPosition().y));
        arect.setRotation(a.getRotation());
        wrect.setPosition(sf::Vector2f(w.getPosition().x, w.getPosition().y));
        wrect.setRotation(w.getRotation());
        psrect.setPosition(sf::Vector2f(ps.getPosition().x, ps.getPosition().y));
        psrect.setRotation(ps.getRotation());
        fleerect.setPosition(sf::Vector2f(flee.getPosition().x, flee.getPosition().y));
        fleerect.setRotation(flee.getRotation());
        evrect.setPosition(sf::Vector2f(ev.getPosition().x, ev.getPosition().y));
        evrect.setRotation(ev.getRotation());
        pathrect.setPosition(sf::Vector2f(pather.getPosition().x, pather.getPosition().y));
        pathrect.setRotation(pather.getRotation());
        hcrect.setPosition(sf::Vector2f(hc.getPosition().x, hc.getPosition().y));
        hcrect.setRotation(hc.getRotation());
        icrect.setPosition(sf::Vector2f(ic.getPosition().x, ic.getPosition().y));
        icrect.setRotation(ic.getRotation());
        offcrect.setPosition(sf::Vector2f(offc.getPosition().x, offc.getPosition().y));
        offcrect.setRotation(offc.getRotation());

        window.clear();
        window.draw(rect);
        window.draw(wrect);
        window.draw(arect);
        window.draw(psrect);
        window.draw(fleerect);
        window.draw(evrect);
        window.draw(pathrect);
        window.draw(hcrect);
        window.draw(icrect);
        window.draw(offcrect);

        for(int i=0; i<flockNum; ++i)
        {
            frects[i].setPosition(flock[i].getPosition().x, flock[i].getPosition().y);
            frects[i].setRotation(flock[i].getRotation());
            window.draw(frects[i]);
        }
        sf::CircleShape temp;
        for(auto i : m_obstacles)
        {
            temp.setFillColor(sf::Color(0.f,0.f,0.f,0.f));
            temp.setOutlineThickness(2.f);
            temp.setOutlineColor(sf::Color(0,255,255,255));
            temp.setPosition(sf::Vector2f(i.getPosition().x, i.getPosition().y));
            temp.setRadius(i.getRadius());
            temp.setOrigin(temp.getRadius(),temp.getRadius());
            window.draw(temp);
        }
        for(auto i : m_walls)
        {
            sf::VertexArray lines(sf::Lines, 2);
            lines[0].color = sf::Color(255,255,255,255);
            lines[0].position = sf::Vector2f(i.From().x, i.From().y);
            lines[1].color = sf::Color(255,255,255,255);
            lines[1].position = sf::Vector2f(i.To().x, i.To().y);
            window.draw(lines);
        }

        window.draw(&drawablePath[0], drawablePath.size(), sf::LineStrip);

        window.display();
    }
    return 0;
}

void createWalls()
{
    //create the walls
    const int NumWallVerts = 4;

    steer::Vector2 walls[NumWallVerts] = {
        steer::Vector2(WALLPADDING, WALLPADDING),
        steer::Vector2(WINDOWX - WALLPADDING, WALLPADDING),
        steer::Vector2(WINDOWX - WALLPADDING, WINDOWY - (WALLPADDING*3)),
        steer::Vector2(WALLPADDING, WINDOWY - (WALLPADDING*3))
    };

    for (int w=0; w<NumWallVerts-1; ++w)
    {
        m_walls.push_back(steer::Wall(true, walls[w], walls[w+1]));
    }

    m_walls.push_back(steer::Wall(true, walls[NumWallVerts-1], walls[0]));
}

void createObstacles()
{
    for (int o=0; o<OBSTACLES; ++o)
    {
        bool bOverlapped = true;

        int NumTrys = 0;
        int NumAllowableTrys = MAXITERS;

        while (bOverlapped)
        {
            NumTrys++;

            if (NumTrys > NumAllowableTrys) return;

            int radius = steer::RandInt(MINRADIUS, MAXRADIUS);

            const int border                 = MINBORDER;
            const int MinGapBetweenObstacles = MINGAP;

            steer::SphereObstacle ob = steer::SphereObstacle(steer::Vector2(steer::RandInt(radius+border, WINDOWX-radius-border),steer::RandInt(radius+border, WINDOWY-radius-30-border)), radius);

            if (!steer::Overlapped(&ob, m_obstacles, MinGapBetweenObstacles))
            {
                m_obstacles.push_back(ob);
                bOverlapped = false;
            }
        }
    }
}
