#include "ColoringRandomPolygon.hpp"
#include "CentralForceFields.hpp"
#include "CollisionPrecision.hpp"
#include "raylib.h"

enum class State { Menu, Polygon, Collision, CentralForceFields };
State state = State::Menu;

static int selectedOption = 0; 
static int menuItemsCount = 3;

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "2D Physics Examples");
    SetTargetFPS(60);

    CFF cff;
    CRP crp;

    while (!WindowShouldClose()) 
    {
        if (state == State::Menu)
        {
            BeginDrawing();
            ClearBackground(BLACK);

            if (IsKeyPressed(KEY_DOWN)) selectedOption = (selectedOption + 1) % menuItemsCount;
            if (IsKeyPressed(KEY_UP)) selectedOption = (selectedOption - 1 + menuItemsCount) % menuItemsCount;

            if (IsKeyPressed(KEY_ENTER))
            {
                switch (selectedOption)
                {
                    case 0: state = State::Polygon; break;
                    case 1: state = State::Collision; break;
                    case 2: state = State::CentralForceFields; break;
                }
            }

            DrawText("Select a State", 300, 100, 30, WHITE);
            DrawText((selectedOption == 0 ? "> Polygon" : "  Polygon"), 320, 200, 25, WHITE);
            DrawText((selectedOption == 1 ? "> Collision" : "  Collision"), 320, 250, 25, WHITE);
            DrawText((selectedOption == 2 ? "> Central Force Fields" : "  Central Force Fields"), 320, 300, 25, WHITE);

            EndDrawing();
        }
        else
        {
            if (IsKeyPressed(KEY_BACKSPACE)) state = State::Menu;

            switch (state)
            {
                case State::Polygon:
                    crp.Render();
                    break;

                case State::Collision:
                    CollisionPrecision::Render();
                    break;

                case State::CentralForceFields:
                    cff.Render();
                    break;

                default:
                    break;
            }
        }

    }

    CloseWindow();
}
