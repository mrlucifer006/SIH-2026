import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def create_collage(output_dir="outputs", collage_file="collage.png"):
    images = [f for f in os.listdir(output_dir) if f.endswith(".png") and f != collage_file]
    if not images:
        print("No images found in outputs directory.")
        return
    
    num_images = len(images)
    cols = 3
    rows = (num_images + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=(15, 5 * rows))
    axes = axes.flatten()
    
    for i, img_name in enumerate(images):
        img_path = os.path.join(output_dir, img_name)
        img = mpimg.imread(img_path)
        axes[i].imshow(img)
        axes[i].set_title(img_name)
        axes[i].axis('off')
        
    for j in range(i + 1, len(axes)):
        axes[j].axis('off')
        
    plt.tight_layout()
    collage_path = os.path.join(output_dir, collage_file)
    plt.savefig(collage_path)
    print(f"Collage saved to {collage_path}")

if __name__ == "__main__":
    create_collage()
