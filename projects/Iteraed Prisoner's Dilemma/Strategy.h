//
//  Strategy.h
//  Iterated Prisoner's Dilemma
//
//  Created by Demarce Williams on 27/11/2023.
//

#ifndef Strategy_h
#define Strategy_h

#include <iostream>
#include <ctime>
#include <cstdlib>
using namespace std;

/****************************
 Strategy Table
 1- Random
 2- Evil
 3-Cooperate
 4-Tit-For-Tat
 
 Possible Moves
 0- Cooperate
 1- Defect
 ***********************************************/



//Strategy Class Definition

class Strategy{
private:
    int currentStrategy; //Stores player strategy
public:
    Strategy(): currentStrategy(1){}//set default strategy to random
    
    /**Accessor**/
    int getStrategy(){//Returns player strategy
        return currentStrategy;
    }//End function
    
    /**Modifier**/
    void setStrategy(int strategy){ //Set player strategy
        currentStrategy = strategy;
    }//End function

    /**Service**/
    
    int cooperateOrDefect(int lastmove){
        int decision(0);
        
        //Make a move based on player's strategy
        switch(currentStrategy){
            case 1: //Random
                decision = rand()%2;
                break;
            case 2: //Evil
                decision = 1;
                break;
            case 3: //Cooperate
                decision = 0;
                break;
            case 4: //Tit-For-Tat
                if (lastmove != -1) {
                    decision = lastmove;
                }else{
                    decision = rand() % 2;
                }//End if
                break;
            default: //Invalid Strategies
                cout << "\nError! Invalid Strategy!\n";
                break;
        }
        return decision;
    }//End function

};

#endif /* Strategy_h */
