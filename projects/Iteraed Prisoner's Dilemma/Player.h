//
//  Player.h
//  Iterated Prisoner's Dilemma
//
//  Created by Demarce Williams on 28/11/2023.
//

#ifndef Player_h
#define Player_h

#include <iostream>
#include <string>
#include "Strategy.h"
using namespace std;

// ID Generator Class Definition
class generatID{
private:
    static int ID; //Stors player ID
public:
    /**Accessor**/
    static int getID(){
        return ID++;
    }//end function
    
    /**Modifier**/
    //Reset Initial ID to zero for new game
    static void resetID(){ //Resets ID to Zero
        ID = 0;
    }//End function
};
//Initialise Static variable
int generatID::ID = 0;

//Player Class Definition
class Player{
private:
    int ID; //Stores player ID
    string name; //Stores player name
    Strategy s; //Stores player stratefy
    int score; //Stores player score
    int * moves; // Store Player moves
    
    int numOfMoves; //Stores number of moves made in the game during the nth iteration
    int maxMoves; //Stores trhe total possible moves the player can make
    int lastMove; //Stores teh opponent's last move
    
    static int numOfPlayers; //Stores number of Player objects
    
public:
    /*********************************************Constructor***************************************************/
    Player(): name(""), score(0), moves(nullptr), numOfMoves(0), maxMoves(0), lastMove(-1){
        ID = generatID::getID(); //Set player ID
        numOfPlayers++; //Count player objects
    }//End function
    
    /*********************************************Copy Constructor***************************************************/
    Player(const Player& other){
        // Copy primitive data members
        ID = other.ID;
        name = other.name;
        score = other.score;
        numOfMoves = other.numOfMoves;
        maxMoves = other.maxMoves;
        lastMove = other.lastMove;

        // Deep copy dynamic array
        if (other.moves != nullptr) {
            moves = other.moves;
        } else {
                moves = nullptr;
        }//End if
        // Copy Strategy (assuming Strategy has an appropriate copy constructor)
        s = other.s;
    }//End function
        

    /****************************************************Accessors******************************************/
    
    int getID(){ //Returns player ID
        return ID;
    }//End Funtion
    
    string getName(){ //Returns player name
        return name;
    }//End Funtion
    
    int getScore(){ //Returns player score
        return score;
    }//End Funtion
    
    string getStrategy(){ //Returns player strategy
        string strategy;
        switch(s.getStrategy()){
            case 1:
                strategy = "Random";
                break;
            case 2:
                strategy = "Evil";
                break;
            case 3:
                strategy = "Cooperate";
                break;
            case 4:
                strategy = "Tit-For-Tat";
                break;
        }//End Switch
        return strategy;
    }//End Funtion
    
    int getLastMove(int x){
        // Check if moves array is not nullptr and x is within bounds
        if (moves != nullptr && x >= 0 && x <= numOfMoves){
            return moves[numOfMoves - (numOfPlayers - 1) + x];
        }else{
            // Handle the error
            cout << "Error: Couldn't get last move!\n";
            return -1;
        }//End if
    }//End function

    
    /***********************************************Modifiers*****************************************************/
    
    void setName(string name){//Set player name
        this-> name = name;
    }//End function
    void resetPlayerInfo(){
        score = 0;
        numOfMoves = 0;
    }
    int * resetMoves(){
        return moves;
    }
    void setScore(int score){//Set player score
        this-> score += score;
    }//End function
    
    void updateStrategy(int strategy){//Set player strategy
        s.setStrategy(strategy);
    }//End function

    void setMaxMoves(int maxMoves){//Set player maximum moves
        this-> maxMoves = maxMoves;
        
        //Allocate memory for store player moves
        moves = new int[this-> maxMoves];
        
    }//End function
    
    void setLastMove(int lastMove){//Set opponent's last move
        this-> lastMove = lastMove;
    }//End if
    
    //Reset Initial ID to zero for new game
    static void resetNumOfPlayers(){ //Resets ID to Zero
        numOfPlayers = 0;
    }//End function


    /*************************************Service Functions********************************************************/
    
    int makeMove(){ //Allows player to make a move and returns that move
        int decision(0);
        //Calls decision function and store return in moves array
        if(moves != nullptr){
            decision = s.cooperateOrDefect(lastMove);
            moves[numOfMoves] = decision;
            numOfMoves++;
        }else{
            cout << "\nError! Could not make move! Storage not found!\n";
        }//End if
        //Returns move
        return decision;
    }//End function
    
    void printMoves(){ //Prints player's moves
        for(int i = 0; i < maxMoves; i++){
            switch(moves[i]){
                case 0:
                    cout << "Cooperate\n";
                    break;
                case 1:
                    cout << "Defect\n";
                    break;
            }//End Switch
        }
    }
    
    /*****************************************Destructor***********************************************************/
    ~Player(){
        //Dynamic memory Deallocation
        delete [] moves;
    }
};
//Initialise Static variable
int Player::numOfPlayers = 0;
 
#endif /* Player_h */
