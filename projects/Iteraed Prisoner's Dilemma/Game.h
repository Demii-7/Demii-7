//
//  Game.h
//  Iterated Prisoner's Dilemma
//
//  Created by Demarce Williams on 28/11/2023.
//

#ifndef Game_h
#define Game_h
#include <iostream>
#include "Player.h"

using namespace std;

#define MAXPLAYERS 2

//Game Class Definition
class Game{
private:
    Player * Players; //Stores player objects
    int numOfRounds; //Stores the number of game rounds
    int numOfPlayers; //Stores number of players in game
public:
    /*****************************************Constructor***********************************************************/
    Game(): Players(nullptr), numOfRounds(0), numOfPlayers(0){}
    
    /*****************************************Accessors***********************************************************/
    
    /*****************************************Modifiers***********************************************************/
    void setNumOfRounds(int rounds){//Set number of rounds/moves in game
        numOfRounds = rounds;
        
        //Set maximum number of moves for each player based on number of rounds
        for(int i = 0; i < numOfPlayers; i++){
            //Reset Player Information if Player chooses to replay teh game with different number of rounds and same information
            Players[i].resetPlayerInfo();
            
            
            Players[i].setMaxMoves(rounds * (numOfPlayers-1));
        }//End for
        
    }//End function
    
    /***************************************Service Functions****************************************************/
    void addPlayer(int numOfPlayers){ //Add players to game
        if(Players == nullptr){
            
            if(numOfPlayers <= MAXPLAYERS){ //Ensures numOfPlayers is within the allowable amount of players
                this-> numOfPlayers = numOfPlayers;
                Players = new Player[this->numOfPlayers];
            }else{
                cout << "\nError! Max reached! Could not add players!\n";
            }//End Inner If
        }else{
            //Executes if user wishes add different players to a new game
            //Deallocate old player objects storage and reset numOfplayers and ID to zero
            delete [] Players;
            generatID::resetID();
            Player::resetNumOfPlayers();
            
            cout << "\n********************************************************\n";
            cout <<"*                         NEW GAME                     *";
            cout << "\n********************************************************\n";

            
            //Allocate memory for new player objects
            this-> numOfPlayers = numOfPlayers;
            Players = new Player[this->numOfPlayers];
        }//End if
    }//End function
    
    void addPlayerToExisting(int numOfPlayersToAdd) {
        if (Players != nullptr) { //Ensures that game already exits
            if (numOfPlayers + numOfPlayersToAdd <= MAXPLAYERS) { // Ensure numOfPlayers is within the allowable amount of players
                // Create a temporary array to hold the updated players
                Player* updatePlayers  = new Player[numOfPlayers + numOfPlayersToAdd];

                // Copy existing players to the updated array
                for (int i = 0; i < numOfPlayers; i++) {
                    updatePlayers[i] = Players[i];
                    updatePlayers[i].resetPlayerInfo();
                }//End for
                
                // Delete the old array and assign the updated array to Players (only if it was previously allocated)
                delete[] Players;
                
                Players = updatePlayers;
                // Update the number of players
                numOfPlayers += numOfPlayersToAdd;
            } else {
                cout << "\nError! Max reached! Could not add players!\n";
            }//End inner if
        } else {
            cout << "\nError! Please create a game first!\n";
        }
    }//End function

    
    void dropPlayer(int playerID) {
        bool isFound = true;
                
        for (int i=0; i<numOfPlayers; i++){
            if(playerID == Players[i].getID()){
                cout << "\nThe player to be removed is found\n" << endl;
                for (int j=i; j<numOfPlayers-1; j++){
                    Players[j] = Players[j+1];
                }//End for
                //Initialise last player object to zero in order to remove duplicate player objects
                Players[numOfPlayers-1] = Player();
                numOfPlayers--;
                isFound = false;
                break;
            }//End if
        }//End for
                
        if(isFound){
            cout << "\nCan't find the player to drop!\n" << endl;
        }else{
            cout << "\nPlayer dropped successfully!\n" << endl;
        }//End if
    }//End function
    
    Player * setPlayerInfo(){//Return player pointer so that Player data can be accessed and modified through the game object
        return Players;
    }//End function
    
    void shareLastMove(int i, int j, int x){
        //Share last moves between current two players
        Players[i].setLastMove(Players[j].getLastMove(x));
        //Share last moves between current two players
        Players[j].setLastMove(Players[i].getLastMove(x));
    }//End function
    
    void Play(){
            int player1, player2;
            //Main Loop to control Game Rounds
            for(int k = 0; k < numOfRounds; k++){
            
                for(int i = 0; i < numOfPlayers; i++){
                    //Used to increment the index to find current player[J]'s last move
                    int x = 0;
                    
                    for(int j = 0; j < i; j++){
                        if(k!=0){
                            shareLastMove(i, j, x);
                        }//End if
                        
                        player1 = Players[i].makeMove();
                        player2 = Players[j].makeMove();
                        x++;
                        
                        if(player1 == 0 && player2 == 0){
                            //Set player scores
                            Players[i].setScore(3);
                            Players[j].setScore(3);
                            
                        }else if(player1 == 1 &&player2 == 1){
                            //Set player scores
                            Players[i].setScore(1);
                            Players[j].setScore(1);
                            
                        }else if(player1 == 1 && player2 == 0){
                            //Set player scores
                            Players[i].setScore(5);
                            Players[j].setScore(0);
                            
                        }else if(player1 == 0 && player2 == 1){
                            //Set player scores
                            Players[i].setScore(0);
                            Players[j].setScore(5);
                            
                        }//End if
                        
                    }//End for
                }//End for
            }//End for

            //Print results
            printResults();
    }//End function
    
    
    void printResults() {
        string strategy;
        int winnerIndex = 0;
        int highestScore = 0;
        
        for (int i = 0; i < numOfPlayers; i++) {
            cout << "\nPlayer ID: " << Players[i].getID() << endl;
            cout << "Name: " << Players[i].getName() << endl;
            cout << "Strategy: " << Players[i].getStrategy() << endl;
            cout<< "Score:   " << Players[i].getScore() << endl << endl;;
            Players[i].printMoves();

            if (Players[i].getScore() > highestScore) {
                highestScore = Players[i].getScore();
                winnerIndex = i;
            }
        }
        cout << "\n**************************************************************\n";
        cout << "*             Congratulations Player " << winnerIndex << "! You won!!!           *\n";
        cout << "**************************************************************\n";

    }//End function


    /*****************************************Destructor***********************************************************/
    
    ~Game(){
        //Dynamically deallocate memory
        delete [] Players;
    }//End function
};
 
#endif /* Game_h */
