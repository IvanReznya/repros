#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "lodepng.h"



unsigned char* load_image(const char* filename, unsigned* width, unsigned* height){
    unsigned char* image = NULL;
    unsigned error = lodepng_decode32_file(&image, width, height, filename);
    
    if(error){
        printf("Ошибка загрузки: %s\n", lodepng_error_text(error));
        return NULL;
    }
    return image;
}


void write_png(const char* filename, const unsigned char* image, unsigned width, unsigned height){
    unsigned char* png;
    unsigned long pngsize;
    unsigned error = lodepng_encode32(&png, &pngsize, image, width, height);
    
    if(error == 0) lodepng_save_file(png, pngsize, filename);
    else printf("Ошибка сохранения: %s\n", lodepng_error_text(error));
    
    free(png);
}


unsigned char* gray_to_rgba(const unsigned char* gray, unsigned width, unsigned height) {
    unsigned char* rgba = malloc(width * height * 4);
    if(!rgba) return NULL;

    for(unsigned i = 0; i < width * height; i++){
        rgba[i * 4] = gray[i];
        rgba[i * 4 + 1] = gray[i];
        rgba[i * 4 + 2] = gray[i];
        rgba[i * 4 + 3] = 255;
    }
    return rgba;
}


unsigned char* convert_to_grayscale(const unsigned char* rgba_image, unsigned width, unsigned height) {
    unsigned char* gray_image = malloc(width * height);
    if(!gray_image) return NULL;
    
    for(unsigned i = 0; i < width * height; i++) {
        unsigned char r = rgba_image[i * 4];
        unsigned char g = rgba_image[i * 4 + 1];
        unsigned char b = rgba_image[i * 4 + 2];
        
        gray_image[i] = (unsigned char)(0.299f * r + 0.587f * g + 0.114f * b);
    }
    return gray_image;
}



void contrast_stretch(unsigned char* gray, unsigned width, unsigned height){
    for(unsigned y = 0; y < height; y++){
            for(unsigned x = 0; x < width; x++){
                unsigned ind = y * width + x;
                gray[ind] = (unsigned char)(250.0 * (gray[ind] - 85) / (600 - 120));
            }
        }

}


void sobel_filter(unsigned char* input, unsigned char* output, unsigned width, unsigned height){
    
    const int Gx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    const int Gy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
    
    for(unsigned y = 1; y < height - 1; y++) {
        for (unsigned x = 1; x < width - 1; x++) {
            int sumGx = 0;
            int sumGy = 0;
            
            for(int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    unsigned idx = (y + ky) * width + (x + kx);
                    sumGx += input[idx] * Gx[ky + 1][kx + 1];
                    sumGy += input[idx] * Gy[ky + 1][kx + 1];
                }
            }
            
            int gradient_magnitude = (int)sqrt(sumGx * sumGx + sumGy * sumGy);
            if (gradient_magnitude > 255) {
                gradient_magnitude = 255;
            }
            output[y * width + x] = (unsigned char)gradient_magnitude;
        }
    }
}


typedef struct {
    int x, y;
} Point;


typedef struct edge{
    int from;
    int to;
    int weight;
}Edge;


typedef struct uni{
    int* parent;
    int* size;
    int count;
}UnionFind;


typedef struct {
    Point* data;
    int front, rear;
    int capacity;
} Queue;


UnionFind* uf_create(int count) {
    UnionFind* uf = (UnionFind*)malloc(sizeof(UnionFind));
    if(!uf) return NULL;
        
    uf->parent = (int*)malloc(count * sizeof(int));
    uf->size = (int*)malloc(count * sizeof(int));
    uf->count = count;

    for(int i = 0; i < count; i++){
        uf->parent[i] = i;
        uf->size[i] = 1;
    }
    return uf;
}


int uf_find(UnionFind* uf, int x){
    if(uf->parent[x] != x){
        uf->parent[x] = uf_find(uf, uf->parent[x]);
    }
    return uf->parent[x];
}


void uf_union(UnionFind* uf, int x, int y) {
    int rootX = uf_find(uf, x);
    int rootY = uf_find(uf, y);

    if(rootX != rootY){
        if(uf->size[rootX] < uf->size[rootY]){
            uf->parent[rootX] = rootY;
            uf->size[rootY] += uf->size[rootX];
        }
        else{
            uf->parent[rootY] = rootX;
            uf->size[rootX] += uf->size[rootY];
        }
    }
}


void add_edge(unsigned char* gray, Edge* edges, int* edge_count, int from, int to) {
    int diff = abs(gray[from] - gray[to]);
    
    edges[*edge_count].from = from;
    edges[*edge_count].to = to;
    edges[*edge_count].weight = diff;
    (*edge_count)++;
}


int compare_edges(const void* a, const void* b) {
    Edge* ea = (Edge*)a;
    Edge* eb = (Edge*)b;
    return ea->weight - eb->weight;
}


void queue_init(Queue* q, int capacity) {
    q->data = (Point*)malloc(capacity * sizeof(Point));
    q->front = 0;
    q->rear = 0;
    q->capacity = capacity;
}


void queue_push(Queue* q, Point p) {
    q->data[q->rear++] = p;
}


Point queue_pop(Queue* q) {
    return q->data[q->front++];
}


int queue_empty(Queue* q) {
    return q->front == q->rear;
}


void queue_free(Queue* q) {
    free(q->data);
}


int pixels_connected(unsigned char* gray, int width, int height, int idx1, int idx2, int threshold) {
    return abs(gray[idx1] - gray[idx2]) <= threshold;
}

void bfs_segmentation(unsigned char* gray, int width, int height, int threshold, UnionFind* uf) {
    int n = width * height;
    char* visited = (char*)calloc(n, sizeof(char));
    if (!visited) return;

    Queue q;
    queue_init(&q, n);

    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < n; i++) {
        if (visited[i] || gray[i] == 0) continue;

        visited[i] = 1;
        queue_push(&q, (Point){i % width, i / width});

        while (!queue_empty(&q)) {
            Point p = queue_pop(&q);
            int idx_p = p.y * width + p.x;

            for (int d = 0; d < 4; d++) {
                int nx = p.x + dx[d];
                int ny = p.y + dy[d];

                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int nidx = ny * width + nx;
                    if (!visited[nidx] && gray[nidx] != 0 && pixels_connected(gray, width, height, idx_p, nidx, threshold)) {
                        visited[nidx] = 1;
                        uf_union(uf, idx_p, nidx);
                        queue_push(&q, (Point){nx, ny});
                    }
                }
            }
        }
    }

    queue_free(&q);
    free(visited);
}



void uf_free(UnionFind* uf) {
    if(!uf) return;
    free(uf->parent);
    free(uf->size);
    free(uf);
}


void color_warm_yellow_5(unsigned char* output_rgb, unsigned char* gray,
                         unsigned width, unsigned height, UnionFind* uf) {

    unsigned char warm_yellow[5][3] = {{255, 255, 204},{255, 229, 153},{255, 204, 102},{255, 178, 51},{255, 153, 0}
    };

    for (unsigned i = 0; i < width * height; i++) {
        if (i == width * height - 4) break;

        int root = uf_find(uf, i);
        int color_idx = root % 5;

        if(gray[i] == 0) {
            output_rgb[i * 4] = 0;
            output_rgb[i * 4 + 1] = 0;
            output_rgb[i * 4 + 2] = 0;
            output_rgb[i * 4 + 3] = 255;
        }
        else{
            output_rgb[i * 4] = warm_yellow[color_idx][0];
            output_rgb[i * 4 + 1] = warm_yellow[color_idx][1];
            output_rgb[i * 4 + 2] = warm_yellow[color_idx][2];
            output_rgb[i * 4 + 3] = 255;
        }
    }
}


int main(){
    const char* input_file = "pich.png";
    const char* output_file = "output.png";

    unsigned width, height;
    unsigned char* output_rgba;
    unsigned char* rgb = load_image(input_file, &width, &height);
    unsigned char* gray = convert_to_grayscale(rgb, width, height);
    unsigned char* filtered = malloc(width * height);
    if(!filtered) printf("Ошибка выделения памяти для фильтра\n");

    contrast_stretch(gray, width, height);
    sobel_filter(gray, filtered, width, height);
    UnionFind* uf = uf_create(width * height);

    int threshold = 10;
    bfs_segmentation(gray, width, height, threshold, uf);
    
    unsigned char* output_rgb = malloc(width * height * 4);
    if(!output_rgb) fprintf(stderr, "Ошибка выделения памяти для output_rgb\n");
    color_warm_yellow_5(output_rgb, gray, width, height, uf);
    write_png(output_file, output_rgb, width, height);
    
    free(rgb);
    free(gray);
    free(output_rgb);
    free(uf->parent);
    free(uf->size);
    free(uf);

    return 0;
}
