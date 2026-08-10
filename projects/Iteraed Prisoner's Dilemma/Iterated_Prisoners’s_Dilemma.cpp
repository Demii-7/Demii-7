//
//  main.cpp
//  Iterated Prisoner's Dilemma
//
//  Created by Demarce Williams on 20/11/2023.
//
/****************************
 The following game reprsents a simulation of teh Iterated Prisoner's Dilemma Gmae. It accepts inputs such as game rounds, player strategies, names etc and allows the user to compete in the game. The game then outputs the results and decalres a winner.
 Strategy Table
 1- Random
 2- Evil
 3-Cooperate
 4-Tit-For-Tat
 
 Possible Moves
 0- Cooperate
 1- Defect
 ***********************************************/


#include <iostream>
#include <string>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <limits>

//Custom Class Headers
#include "Strategy.h"
#include "Player.h"
#include "Game.h"

#define MAXPLAYERS 2
using namespace std;

//Function Prototypes
bool isValidInput();

int main() {
    //Seed random number generation
    srand(static_cast<unsigned int>(time(NULL)));

    //Declare objects
    int choice(0);
    int method(0);
    int totalPlayers(0);
    int addplayers(0);
    int numOfRounds(0);
    string name("");
    bool indicator = true;
    bool indicator2 = true;
    bool indicator3= true;
    
    //Declare Game object
    Game g;

    //Main menu loop
    do{
        // Program title and Welcome message
        cout << "\n                     ITERATED PRISONER'S DILEMMA\n";
        cout << "\n              Welcome to the Iterated Prisoner's Dilemma Game.\n";
        cout << "\n_____________________________________________________________________________\n";
        cout << "\nPlease see the menu that follow. Ensure to set all parameters before proceeding.\n";
        
        // Prints menu to user
        cout << "\n1  : Add/Drop Players.\n";
        cout << "2  : Set Number of Rounds.\n";
        cout << "3  : Choose Strategy.\n";
        cout << "4  : START\n";
        cout << "5  : Exit Program.\n";
        
        
        //Ensures user input is of integer type
        do{
            // Prompts the user to make a selection from menu
            cout << "\nPlease make a selection from the menu above: ";
            cin >> choice;
        }while(!isValidInput());
        
        
        //Switch structure to process user selection
        switch(choice){
            case 1:{//Creates player objects
                
                int selection(0);
                //Ensures user input is of integer type and is present in the menu
                do{
                    // Prints menu to user
                    cout << "\n1: Add Players to New Game\n";
                    cout << "2: Add Players to Existing\n";
                    cout << "3: Drop Players.\n";
                    cout << "4: Return to Main Menu.\n";
                    do{
                        // Prompts the user to make a selection from menu
                        cout << "\nPlease make a selection from the menu above: ";
                        cin >> selection;
                        if(selection < 1|| selection > 4){
                            cout << "\n\nError! Please choose an option in the menu!!\n\n";
                        }//End if
                        
                    }while(!isValidInput() || selection < 1 || selection > 4);
                    
                    switch(selection){
                        case 1:{ //User chooses to add players
                            //Ensures that maximum allowable players is not exceeded
                            do{
                                cout << "\nEnter the number of players: ";
                                cin >> totalPlayers;
                                
                                if(totalPlayers > MAXPLAYERS){
                                    cout << "\nERROR | Must have a maximum of 2 players!\n";
                                }else if (totalPlayers < 2){
                                    cout << "\nERROR | Minimum of 2 players required to start game!\n";
                                }//End if
                                
                            }while(!isValidInput() || totalPlayers < 2 || totalPlayers > MAXPLAYERS);
                            //Adds players to game
                            g.addPlayer(totalPlayers);
                            //Clears input buffer up to newline character
                            cin.ignore();
                            for(int i = 0; i < totalPlayers; i++){
                                cout << "\nEnter the name of Player "<< i << " : ";
                               getline(cin,name); //Allows for names with white spaces
                                //Stores name in player object
                                g.setPlayerInfo()[i].setName(name);
                            }//End for
                            
                            //Confirms Creation of Players
                            cout << "\nPlayers added successfully!\n";
                            indicator = false;
                            break;
                        }case 2:{
                            if(indicator){
                                cout << "\nERROR! Please create a game first!\n";
                                continue;
                            }//End if
                            bool maxReached = false;
                            //Ensures that maximum allowable players is not exceeded
                            do{
                                cout << "\nEnter the number of players to add: ";
                                cin >> addplayers;
                                
                                if(addplayers < 1){
                                    cout << "\nERROR | Must add at least 1 player!\n";
                                }else if ((totalPlayers + addplayers) > MAXPLAYERS){
                                    cout << "\nERROR | Must have a maximum of 2 players!\n";
                                    maxReached = true;
                                    break;
                                }//End if
                            }while(!isValidInput() || addplayers < 1 || (totalPlayers + addplayers) > MAXPLAYERS);
                            
                            if(!maxReached){ //Executes only if number of players being added is within limit
                                //Adds players to game
                                g.addPlayerToExisting(addplayers);
                                //Clears input buffer up to newline character
                                cin.ignore();
                                for(int i = totalPlayers; i < totalPlayers + addplayers; i++){
                                    cout << "\nEnter the name of Player "<< i << " : ";
                                    getline(cin,name); //Allows for names with white spaces
                                    //Stores name in player object
                                    g.setPlayerInfo()[i].setName(name);
                                }//End for
                                
                                //Confirms Creation of Players
                                cout << "\nPlayers added successfully!\n";
                                totalPlayers += addplayers;
                            }else{
                                continue;
                            }//End if
                            break;
                        }case 3:{
                            if(indicator){
                                cout << "\nERROR! Please create a game first!\n";
                                continue;
                            }//End if

                            int playerID(0);
                            //Ensures user input is of integer type
                            do{
                                // Prompts the user to make a selection from menu
                                cout << "\nPlease enter the ID of the Player you wish to drop: ";
                                cin >> playerID;
                            }while(!isValidInput());
                            
                            g.dropPlayer(playerID);
                            totalPlayers--; // Account for player dropped
                            break;
                        }case 4:{
                            cout <<"\nReturning to Main Menu ....\n";
                            break;
                        }default:{
                            cout << "\n\nError! Please choose an option in the menu!!\n\n";
                            break;
                        }//End of case block
                    }//End Switch
                }while(selection != 4);
                break;
                
            }case 2:{
                //Ensures players are created before proceeding to this option
                if(indicator){
                    cout << "\nERROR! Please ensure players are added!\nYou will NOT proceed until all parameters are set!";
                    continue;
                }//End if
                bool flag = true;
                
                if(!flag){
                    for(int i = 0; i < totalPlayers; i++){
                        delete [] g.setPlayerInfo()[i].resetMoves();
                    }//End for
                    flag = true;
                }//End if
                do{
                    //Set Number of Rounds for game
                    cout << "\nEnter the number of rounds: ";
                    cin >> numOfRounds;
                    
                    if(numOfRounds < 0){
                        cout << "\nError! Number of rounds cannot be negative!\n";
                    }//End if
                    
                }while(!isValidInput() || numOfRounds < 0);
                g.setNumOfRounds(numOfRounds);
                
                //Confirms that the number of rounds are set
                indicator2 = false;
                flag = false;
                break;
            }case 3:{
                //Ensures array is created before proceeding to this option
                if(indicator || indicator2){
                    cout << "\nERROR! Please ensure players are added and number of rounds are set!\nYou will NOT proceed until all parameters are set!";
                    continue;
                }//End if
                
                //Choose strategy for each player
                cout << "\nPlease choose a strategy from the menu that follow.\n";
                
                // Prints menu to user
                cout << "\n1  : Random.\n";
                cout << "2  : Evil.\n";
                cout << "3  : Cooperate.\n";
                cout << "4  : Tit-for-Tat.\n";
                
                for(int i = 0; i < totalPlayers; i++){
                    //Ensures user input is of integer type
                    do{
                        // Prompts the user to make a selection from menu
                        cout << "\nPlease choose Player " << i << " strategy: ";
                        cin >> method;
                        
                        if(method < 1 || method > 4 ){
                            cout << "\nError! Not an option! Choose from the menu!\n";
                        }//End if
                    }while(!isValidInput() || method < 1 || method > 4 );
                    
                    //Update Player strategy
                    g.setPlayerInfo()[i].updateStrategy(method);
                }//End for
                
                //Confirms creation of players and
                indicator3 = false;
                break;
            }case 4:{
                //Ensures all previous options are chosen and preparations are made before starting
                if(indicator || indicator2 || indicator3){
                    cout << "\nERROR! Please ensure to add players, set the number of rounds!\nYou will NOT proceed until all parameters are set!";
                    continue;
                }//End if
                
                //StartGame
                g.Play();

                break;
            }case 5:{
                //Exit messages
                cout << "\nThank you for playing...\n";
                cout << "Goodbye!\n" << endl;
                break;
                
            }default:{
                cout << "\n\nError! Please choose an option in the menu!!\n\n";
                continue;
            }//End of caseblock
        }//End of switch statement

    }while(choice != 5);
    
    
    return 0;
}//End main

//Function Definitions
bool isValidInput(){
    bool validInput = true;
    
    //Checks Validity of input
    if(cin.fail()){
        
        validInput = false;
        cout << "\nError! Invalid input! Please enter a valid input!\n";
        
        //clears characters in i
        //Clear input buffer up to the limit or upon encountering a newline character.
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(),'\n');
    }//End if
    
    //Return outcome of input validation true- valid false- invalid
    return validInput;
}//End of function

