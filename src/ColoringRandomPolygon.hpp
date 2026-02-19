#pragma once

#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

struct Circle {
    Vector2 position;
    Vector2 velocity;
    float radius;
};

struct Polygon {
    std::vector<Vector2> vertices;
    std::vector<Color> edgeColors; 
    std::vector<float> edgeWidths; 
    float rotation; 
};

inline static std::vector<float> GenerateEdgeWidths(int sides, float minWidth, float maxWidth) {
    if (minWidth > maxWidth) {
        throw std::invalid_argument("minWidth cannot be greater than maxWidth.");
    }

    std::vector<float> edgeWidths;
    for (int i = 0; i < sides; i++) {
        float randomWidth = minWidth + (GetRandomValue(0, 100) / 100.0f) * (maxWidth - minWidth);
        edgeWidths.push_back(randomWidth);
    }
    return edgeWidths;
}

inline static Polygon GeneratePolygon(int sides, const std::vector<float>& edgeWidths, Vector2 center) {
    if (sides != edgeWidths.size()) {
        throw std::invalid_argument("Number of sides must match the number of edge widths.");
    }

    Polygon polygon;
    float angleStep = 2 * PI / sides;

    for (int i = 0; i < sides; i++) {
        float angle = i * angleStep;
        float radius = edgeWidths[i]; 
        polygon.vertices.push_back({
            center.x + radius * cos(angle),
            center.y + radius * sin(angle)
        });
        polygon.edgeColors.push_back(BLACK); 
    }
    polygon.edgeWidths = edgeWidths;
    polygon.rotation = 0.0f;
    return polygon;
}

inline static void RotatePolygon(Polygon& polygon, float angle) {
    Vector2 center = {0, 0};
    for (const auto& vertex : polygon.vertices) {
        center = Vector2Add(center, vertex);
    }
    center = Vector2Scale(center, 1.0f / polygon.vertices.size());

    for (auto& vertex : polygon.vertices) {
        Vector2 relative = Vector2Subtract(vertex, center);
        float rotatedX = relative.x * cos(angle) - relative.y * sin(angle);
        float rotatedY = relative.x * sin(angle) + relative.y * cos(angle);
        vertex = Vector2Add(center, {rotatedX, rotatedY});
    }
    polygon.rotation += angle;
}

inline static void HandleCollision(Circle& circle, Polygon& polygon, RenderTexture2D& renderTexture, std::vector<Vector2>& tracePath) {
    for (size_t i = 0; i < polygon.vertices.size(); i++) {
        Vector2 start = polygon.vertices[i];
        Vector2 end = polygon.vertices[(i + 1) % polygon.vertices.size()];

        Vector2 edge = Vector2Subtract(end, start);
        Vector2 normal = {-edge.y, edge.x};
        float length = sqrt(normal.x * normal.x + normal.y * normal.y);
        normal = Vector2Scale(normal, 1.0f / length);

        Vector2 toCircle = Vector2Subtract(circle.position, start);

        float dist = Vector2DotProduct(normal, toCircle);
        if (fabs(dist) <= circle.radius) {
            circle.velocity = Vector2Subtract(circle.velocity, Vector2Scale(normal, 2 * Vector2DotProduct(circle.velocity, normal)));
            circle.position = Vector2Add(circle.position, Vector2Scale(normal, circle.radius - dist));
            polygon.edgeColors[i] = RED;

        }
    }
}

inline static float CalculatePolygonArea(const Polygon& polygon) {
    float area = 0.0f;
    int n = polygon.vertices.size();

    for (int i = 0; i < n; i++) {
        Vector2 v1 = polygon.vertices[i];
        Vector2 v2 = polygon.vertices[(i + 1) % n]; // Next vertex (wraps around)
        area += (v1.x * v2.y - v1.y * v2.x);
    }

    return fabs(area) / 2.0f; // Shoelace formula
}

inline static bool IsPointInsidePolygon(Vector2 point, const std::vector<Vector2>& vertices) {
    int count = vertices.size();
    bool inside = false;
    for (int i = 0, j = count - 1; i < count; j = i++) {
        if (((vertices[i].y > point.y) != (vertices[j].y > point.y)) &&
            (point.x < (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

inline static float CalculateColoredArea(const Polygon& polygon, RenderTexture2D& renderTexture) {
    Image image = LoadImageFromTexture(renderTexture.texture);
    Color* pixels = LoadImageColors(image);

    int coloredPixels = 0;
    for (int y = 0; y < image.height; y++) {
        for (int x = 0; x < image.width; x++) {
            int i = y * image.width + x;
            if (pixels[i].r == YELLOW.r && pixels[i].g == YELLOW.g && pixels[i].b == YELLOW.b) {
                if (IsPointInsidePolygon({(float)x, (float)y}, polygon.vertices)) {
                    coloredPixels++;
                }
            }
        }
    }

    float area = (float)coloredPixels;
    UnloadImageColors(pixels);
    UnloadImage(image);
    return area;
}

inline static float CalculateColoredArea(RenderTexture2D& renderTexture) {
    Image image = LoadImageFromTexture(renderTexture.texture);
    Color* pixels = LoadImageColors(image);

    int coloredPixels = 0;
    for (int i = 0; i < image.width * image.height; i++) {
        if (pixels[i].r == YELLOW.r && pixels[i].g == YELLOW.g && pixels[i].b == YELLOW.b) {
            coloredPixels++;
        }
    }

    float area = (float)coloredPixels; // Total number of affected pixels
    UnloadImageColors(pixels);
    UnloadImage(image);
    return area;
}

constexpr int screenWidth = 800;
constexpr int screenHeight = 600;


struct CRP
{

  CRP() : sides(7)
  {
      circle = {{400, 300}, {15, -3}, 10}; 

      edgeWidths = GenerateEdgeWidths(sides, 150.0f, 250.0f);
      polygon = GeneratePolygon(sides, edgeWidths, {400, 300});

      polygonArea = CalculatePolygonArea(polygon);

      renderTexture = LoadRenderTexture(screenWidth, screenHeight);
      BeginTextureMode(renderTexture);
      ClearBackground(BLANK);
      EndTextureMode();
  }

  ~CRP() { UnloadRenderTexture(renderTexture); }

  inline void Render()
  {

      if (!allColored) {
          timer += GetFrameTime();
          tracePath.push_back(circle.position); 
      }

      circle.position.x += circle.velocity.x;
      circle.position.y += circle.velocity.y;

      HandleCollision(circle, polygon, renderTexture, tracePath);

      BeginTextureMode(renderTexture);
      for (size_t j = 0; j < tracePath.size() - 1; j++) {
          DrawLineEx(tracePath[j], tracePath[j + 1], circle.radius * 2.0f, YELLOW);
      }
      EndTextureMode();
      float coloredArea = CalculateColoredArea(renderTexture);
      
      float coloredPercentage = (coloredArea / polygonArea) * 100.0f;
      allColored = (coloredPercentage >= 90.0f);

      if (IsKeyDown(KEY_Q)) RotatePolygon(polygon, -0.05f);
      if (IsKeyDown(KEY_E)) RotatePolygon(polygon, 0.05f);

      if (allColored && IsKeyPressed(KEY_R)) {
          edgeWidths = GenerateEdgeWidths(sides, 150.0f, 250.0f); 
          polygon = GeneratePolygon(sides, edgeWidths, {400, 300}); 
          timer = 0.0f;
          allColored = false;
          tracePath.clear();
          polygonArea = CalculatePolygonArea(polygon);

          BeginTextureMode(renderTexture);
          ClearBackground(BLANK);
          EndTextureMode();
      }

      BeginDrawing();
      ClearBackground(RAYWHITE);

      for (size_t i = 0; i < polygon.vertices.size(); i++) {
          Vector2 start = polygon.vertices[i];
          Vector2 end = polygon.vertices[(i + 1) % polygon.vertices.size()];
          DrawLineV(start, end, polygon.edgeColors[i]);
      }

      DrawCircleV(circle.position, circle.radius, RED);

      DrawTextureRec(renderTexture.texture, {0, 0, (float)renderTexture.texture.width, -(float)renderTexture.texture.height}, {0, 0}, WHITE);

      DrawText(TextFormat("Time: %.2f seconds", timer), 10, 10, 20, BLACK);
      DrawText(TextFormat("Colored Percentage: %.2f%%", coloredPercentage), 10, 40, 20, BLACK);
      DrawText(TextFormat("Colored Pixels: %.2f", coloredArea), 10, 60, 20, BLACK);
      DrawText(TextFormat("Polygon Pixels: %.2f", polygonArea), 10,80, 20, BLACK);

      DrawText("BACKSPACE to return", 10, GetScreenHeight() - 25, 20, BLACK);
      if (allColored) {
          DrawText("90% of the polygon is colored!", screenWidth / 2 - 120, screenHeight / 2, 20, GREEN);
          DrawText("Press 'R' to reset!", screenWidth / 2 - 100, screenHeight / 2 + 30, 20, GREEN);
      }

      EndDrawing();
  }


private:
  Polygon polygon;
  Circle circle;
  float timer = 0.0f;
  bool allColored = false;
  std::vector<Vector2> tracePath;
  float polygonArea;
  RenderTexture2D renderTexture;
  std::vector<float> edgeWidths;
  const int sides;
};


