import tkinter as tk
from tkinter import ttk
from tkinter import simpledialog
from PIL import Image, ImageTk, ImageDraw, ImageFont
import networkx as nx
from IPython import embed
import argparse
from PIL import Image
import os 
import json
import secrets
import numpy as np
from omegaconf import OmegaConf

class SceneGraphBuilder:
    def __init__(self, root, zoom_in,zoom_out,image_path, scgraph_path, out_path, node_types, edge_types,font_size=5):
        self.root = root
        self.root.title("Scene Graph Builder")
        # Load image
        self.image = Image.open(image_path).convert('RGB')
        width,height = self.image.size
        self.zoom = zoom_in
        self.zoom_out = zoom_out
        self.unscaled_image = self.image.copy()
        self.image = self.image.resize((round(width*self.zoom),round(height*self.zoom)))
        # print(self.image.size)
        self.image_tk = ImageTk.PhotoImage(self.image)
        # Create scrollable canvas with fixed window size
        self.frame = tk.Frame(root)
        self.frame.pack(fill=tk.BOTH, expand=True)

        # Fixed canvas size (800x600) regardless of image size
        self.canvas_width = 800
        self.canvas_height = 600

        self.canvas = tk.Canvas(self.frame, width=self.canvas_width, height=self.canvas_height,
                               scrollregion=(0, 0, self.image.size[0], self.image.size[1]))

        # Add scrollbars
        v_scrollbar = tk.Scrollbar(self.frame, orient=tk.VERTICAL, command=self.canvas.yview)
        h_scrollbar = tk.Scrollbar(self.frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        self.canvas.configure(xscrollcommand=h_scrollbar.set, yscrollcommand=v_scrollbar.set)

        # Grid layout for canvas and scrollbars
        self.canvas.grid(row=0, column=0, sticky='nswe')
        v_scrollbar.grid(row=0, column=1, sticky='ns')
        h_scrollbar.grid(row=1, column=0, sticky='ew')

        # Configure grid weights
        self.frame.grid_rowconfigure(0, weight=1)
        self.frame.grid_columnconfigure(0, weight=1)

        # Display image on canvas
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image_tk)
        # Use original resolution for export
        self.output_image = self.unscaled_image.copy()
        self.output_image_draw = ImageDraw.Draw(self.output_image,mode='RGBA')
        # print(self.output_image_draw.mode)
        # Initialize NetworkX graph
        self.graph = nx.Graph()

        # Node tracking
        self.nodes = {}
        self.node_types = node_types
        self.edge_types = edge_types
        self.edges = []

        # Bind mouse events
        self.canvas.bind("<Double-Button-1>", self.add_node)
        self.canvas.bind("<Button-1>", self.start_edge)
        self.canvas.bind("<B1-Motion>", self.drag_edge)
        self.canvas.bind("<ButtonRelease-1>", self.end_edge)
        self.canvas.bind("<Key-q>", self.quit)  # Press 'q' to quit

        # Scroll wheel for panning
        self.canvas.bind("<MouseWheel>", self.pan_vertical)
        self.canvas.bind("<Button-4>", self.pan_vertical)  # Linux scroll up
        self.canvas.bind("<Button-5>", self.pan_vertical)  # Linux scroll down

        # Alt + scroll wheel for horizontal panning
        self.canvas.bind("<Alt-MouseWheel>", self.pan_horizontal)
        self.canvas.bind("<Alt-Button-4>", self.pan_horizontal)  # Linux alt+scroll up
        self.canvas.bind("<Alt-Button-5>", self.pan_horizontal)  # Linux alt+scroll down

        # +/- keys for zoom
        root.bind("<Key-plus>", self.zoom_in_key)
        root.bind("<Key-equal>", self.zoom_in_key)  # For US keyboards where + is shift+=
        root.bind("<Key-minus>", self.zoom_out_key)
        root.bind("<Key-KP_Add>", self.zoom_in_key)  # Numpad +
        root.bind("<Key-KP_Subtract>", self.zoom_out_key)  # Numpad -

        root.bind("<Alt-s>",self.save_canvas_as_img)

        # Make sure canvas can receive focus for key events
        self.canvas.focus_set()
        self.fine_node_counter = 64
        self.current_start_node = None
        self.zoomed_images = {}
        self.zoomed_images_draw = {}
        self.img_save_path = out_path
        self.image_origin = np.array([0.0,0.0])
        self.zimgfont = "nimbus"
        self.pimgfont = "nimbus"
        
        # Calculate adaptive sizes based on image dimensions
        # Use the smaller dimension to ensure everything fits well
        min_dimension = min(width, height)

        # Base sizes scale with image size for legibility
        # For small images (250px), these will be smaller
        # For large images (1500px), these will be larger
        # print(f'Image min dimension: {min_dimension}')
        base_font_size = font_size #max(2, min(10, min_dimension // 75))  # 4-10 pixel range (much smaller)
        self.font_size = font_size
        # print(f'Base font size: {base_font_size}')
        # Rectangle should be large enough to contain the text
        base_rect_size = max(6, base_font_size * 2)  # At least 2x font size for text to fit

        # Canvas display sizes (scale with zoom for visibility)
        self.canvas_rect_size = round(base_rect_size * self.zoom)
        self.canvas_font_size = round(base_font_size * self.zoom)
        #print(f"canvas text size: {self.canvas_font_size}")
        # Export image sizes (adaptive to image size for legibility)
        self.export_rect_size = base_rect_size
        self.export_font_size = base_font_size
        try:
            self.export_font = ImageFont.truetype("arial.ttf", self.export_font_size)
        except OSError:
            # Fallback to default font if arial.ttf not found
            self.export_font = ImageFont.load_default()
        self.node_names = []
        self.scgraph_save_path = scgraph_path
    
    def quit(self, event):
        self.root.destroy()
        exit()
    
    def save_canvas_as_img(self, event):
        print("Saving image...")

        # Create a canvas-sized image with the current view
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        # Get the current scroll region and visible area
        x1 = self.canvas.canvasx(0)
        y1 = self.canvas.canvasy(0)
        x2 = self.canvas.canvasx(canvas_width)
        y2 = self.canvas.canvasy(canvas_height)

        # Create image from the current zoomed image
        current_image = self.image.copy()

        # Crop to visible area
        crop_x1 = max(0, int(x1))
        crop_y1 = max(0, int(y1))
        crop_x2 = min(current_image.width, int(x2))
        crop_y2 = min(current_image.height, int(y2))

        visible_image = current_image.crop((crop_x1, crop_y1, crop_x2, crop_y2))

        # Create a new image to draw annotations on
        canvas_view_image = visible_image.copy()
        draw = ImageDraw.Draw(canvas_view_image, mode='RGBA')

        # Draw all visible edges FIRST (so they appear behind nodes)
        for start_node, end_node in self.edges:
            if start_node in self.nodes and end_node in self.nodes:
                start_x, start_y = self.nodes[start_node]
                end_x, end_y = self.nodes[end_node]

                # Check if edge is at least partially visible
                if (crop_x1 <= start_x <= crop_x2 and crop_y1 <= start_y <= crop_y2) or \
                   (crop_x1 <= end_x <= crop_x2 and crop_y1 <= end_y <= crop_y2):
                    # Adjust coordinates relative to the cropped image
                    rel_start_x = start_x - crop_x1
                    rel_start_y = start_y - crop_y1
                    rel_end_x = end_x - crop_x1
                    rel_end_y = end_y - crop_y1

                    draw.line([rel_start_x, rel_start_y, rel_end_x, rel_end_y], fill="red", width=3)

        # Draw all visible nodes ON TOP (so they appear in front of edges)
        for name, (node_x, node_y) in self.nodes.items():
            # Check if node is in visible area
            if crop_x1 <= node_x <= crop_x2 and crop_y1 <= node_y <= crop_y2:
                # Adjust coordinates relative to the cropped image
                rel_x = node_x - crop_x1
                rel_y = node_y - crop_y1

                # Draw rectangle
                adjusted_rect_size = max(1, round(self.canvas_rect_size * 0.5))  # Scale down for cropped view
                draw.rectangle(
                    (rel_x - adjusted_rect_size, rel_y - adjusted_rect_size,
                     rel_x + adjusted_rect_size, rel_y + adjusted_rect_size),
                    fill='yellow', width=3, outline='blue'
                )

                # Draw text
                # Adjust PIL font size to match tkinter rendering (PIL tends to render smaller)
                adjusted_font_size = max(1, round(self.canvas_font_size * 2.0))  # 20% larger to match tkinter
                try:
                    font = ImageFont.truetype("arial.ttf", adjusted_font_size)
                except OSError:
                    try:
                        # Try default PIL font with adjusted size
                        font = ImageFont.load_default()
                    except:
                        font = ImageFont.load_default()

                draw.text((rel_x, rel_y), name, anchor='mm', fill='black', font=font)

        # Save the canvas view image
        canvas_view_image.save(self.img_save_path)
        #print(f"Canvas view saved to: {self.img_save_path}")

        # Also save the original resolution image with annotations (for coordinate reference)
        self.output_image.save(self.img_save_path.replace('.png', '_original_res.png'))

        # Save the graph data
        graph = self.graph.copy()
        graph = nx.Graph(graph).to_undirected()
        serialized_graph = nx.node_link_data(graph)
        print(serialized_graph)
        with open(os.path.join(self.scgraph_save_path), "w") as f:
            json.dump(serialized_graph, f, indent=4)   
    
    def add_node(self, event):
        #print("Add Node")
        # Convert canvas click coordinates to actual image coordinates
        # Account for scroll position and zoom
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)
        x, y = canvas_x, canvas_y
        dialog_msg = """\nEnter the node's type: \n"""
        for i,n in enumerate(self.node_types):
            dialog_msg += f"\n{i+1}:{n}"
        dialog_msg+='\n'
        node_name = None #input("Enter the node's name, e.g. 'kitchen' (optional)\n")#simpledialog.askstring("Node Name", "Enter the node's name, e.g. 'Kitchen' (optional):")
        while True:
            try:
                node_type = int(input(dialog_msg))#simpledialog.askinteger("Node Type", dialog_msg)
                # print(node_type)
                break
            except Exception as e:
                print("Invalid node type, try again")
                
        
        random_node_name = ''
        if node_name == None:
            node_name = ''
            while True:
                random_node_name = secrets.token_hex(1) #random 4 digit alphanumeric
                if random_node_name not in self.node_names:
                    self.node_names.append(random_node_name)
                    break
        
        full_node_name = random_node_name + node_name
        if node_type and 0<node_type<len(self.node_types)+1:
            # Add node to graph
            # print((x,y))
            self.graph.add_node(full_node_name, type=self.node_types[node_type-1], pos=(round(x/self.zoom), round(y/self.zoom)))
            
            self.nodes[full_node_name] = (x, y)

            # Display node visually on canvas (scaled for current zoom)
            self.canvas.create_rectangle(x - self.canvas_rect_size, y - self.canvas_rect_size, x + self.canvas_rect_size, y + self.canvas_rect_size, fill='yellow', width = 3, outline="blue")
            self.canvas.create_text(x, y, text=full_node_name,font=(self.pimgfont, self.canvas_font_size), fill="black")
            #print(f'x-:{x - self.parent_oval_size}, y-:{y - self.parent_oval_size}')
            #print(f'x+:{x + self.parent_oval_size}, y+:{y + self.parent_oval_size}')
            #print(f'full node name:{full_node_name}')
            
            # Draw on export image at original resolution coordinates
            orig_x = round(x / self.zoom)
            orig_y = round(y / self.zoom)
            self.output_image_draw.rectangle(
                (
                 orig_x - self.export_rect_size, orig_y - self.export_rect_size,
                 orig_x + self.export_rect_size, orig_y + self.export_rect_size
                ),fill='yellow',width=3,outline='blue'
                )
            self.output_image_draw.text(
                (orig_x, orig_y), full_node_name, anchor='mm', fill='black', font=self.export_font)
            
    def start_edge(self, event):
        #print("Start Edge")
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)
        self.current_start_node = self.get_nearest_node(canvas_x, canvas_y)

    def drag_edge(self, event):
        #print("Drag Edge")
        if self.current_start_node:
            canvas_x = self.canvas.canvasx(event.x)
            canvas_y = self.canvas.canvasy(event.y)
            self.canvas.delete("temp_edge")
            self.canvas.create_line(*self.nodes[self.current_start_node], canvas_x, canvas_y, tags="temp_edge", dash=(2, 2),width=5)
    
    def end_edge(self, event):
        #print("End Edge")
        self.canvas.delete("temp_edge")
        edge_type = None
        if self.current_start_node:
            canvas_x = self.canvas.canvasx(event.x)
            canvas_y = self.canvas.canvasy(event.y)
            end_node = self.get_nearest_node(canvas_x, canvas_y)
            if end_node and end_node != self.current_start_node:
                
                dialog_msg = """\nEnter the Edge's type: \n"""
                for i,n in enumerate(self.edge_types):
                    dialog_msg += f"\n{i+1}:{n}"
                dialog_msg+='\n'
                #edge_type = simpledialog.askinteger("Node Type", dialog_msg)
                try:
                    edge_type = int(input(dialog_msg))
                except Exception as e:
                    return
                #print(edge_type)
                #print(self.edge_types[edge_type-1])
                self.graph.add_edge(self.current_start_node, end_node,type = self.edge_types[edge_type-1])
                self.edges.append((self.current_start_node, end_node))
                current_start_node = self.nodes[self.current_start_node]
                # Draw edge on export image at original resolution
                start_orig = (round(current_start_node[0]/self.zoom), round(current_start_node[1]/self.zoom))
                end_orig = (round(self.nodes[end_node][0]/self.zoom), round(self.nodes[end_node][1]/self.zoom))
                self.output_image_draw.line([*start_orig, *end_orig], fill="red", width=3)

                # Redraw canvas to maintain proper layering (edges behind nodes)
                self.redraw_canvas()
                
                #print([*self.nodes[self.current_start_node], event.x, event.y])
            self.current_start_node = None                  
            
    def move_from(self, event):
        ''' Remember previous coordinates for scrolling with the mouse '''
        self.image_origin[0]-=event.x
        self.image_origin[1]-=event.y

    def move_to(self, event):
        ''' Drag (move) canvas to the new position '''
        self.canvas.scan_dragto(event.x, event.y, gain=1)
        #self.show_image()  # redraw the image

    def get_nearest_node(self, x, y):
        #print("Getting nearest Node")
        min_distance = float('inf')
        nearest_node = None

        for name, (nx, ny) in self.nodes.items():
            distance = ((nx - x) ** 2 + (ny - y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_node = name
        #print(f'Nearest Node:{nearest_node}')
        return nearest_node

    def pan_vertical(self, event):
        """Handle vertical panning with scroll wheel"""
        if event.delta > 0 or event.num == 4:  # Scroll up
            self.canvas.yview_scroll(-1, "units")
        else:  # Scroll down
            self.canvas.yview_scroll(1, "units")

    def pan_horizontal(self, event):
        """Handle horizontal panning with Alt + scroll wheel"""
        if event.delta > 0 or event.num == 4:  # Scroll left
            self.canvas.xview_scroll(-1, "units")
        else:  # Scroll right
            self.canvas.xview_scroll(1, "units")

    def zoom_in_key(self, event):
        """Handle zoom in with + key"""
        zoom_factor = 1.1
        new_zoom = self.zoom * zoom_factor

        # Limit zoom range
        if new_zoom <= 5.0:
            self.zoom = new_zoom
            self.update_zoom()

    def zoom_out_key(self, event):
        """Handle zoom out with - key"""
        zoom_factor = 0.9
        new_zoom = self.zoom * zoom_factor

        # Limit zoom range
        if new_zoom >= 0.1:
            self.zoom = new_zoom
            self.update_zoom()

    def update_zoom(self):
        """Update canvas display sizes and redraw"""
        # Update canvas display sizes using the adaptive base sizes
        width, height = self.unscaled_image.size
        min_dimension = min(width, height)
        base_font_size = self.font_size#max(4, min(10, min_dimension // 50))  # 4-10 pixel range (much smaller)
        base_rect_size = max(12, base_font_size * 2)  # At least 2x font size for text to fit

        self.canvas_rect_size = round(base_rect_size * self.zoom)
        self.canvas_font_size = round(base_font_size * self.zoom)
        #print(f"canvas text size: {self.canvas_font_size}")
        # Redraw the image and all annotations
        self.redraw_canvas()

    def redraw_canvas(self):
        """Redraw the entire canvas with new zoom level"""
        # Clear canvas
        self.canvas.delete("all")

        # Resize and redraw image
        width, height = self.unscaled_image.size
        self.image = self.unscaled_image.resize((round(width*self.zoom), round(height*self.zoom)))
        self.image_tk = ImageTk.PhotoImage(self.image)

        # Update scroll region to match new image size
        self.canvas.configure(scrollregion=(0, 0, self.image.size[0], self.image.size[1]))

        # Display image
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image_tk)

        # Redraw all nodes - positions need to be scaled to current zoom
        updated_nodes = {}
        for name, node_data in self.graph.nodes(data=True):
            if 'pos' in node_data:
                # Get original position and scale to current zoom
                orig_x, orig_y = node_data['pos']
                x = round(orig_x * self.zoom)
                y = round(orig_y * self.zoom)

                # Update stored position
                updated_nodes[name] = (x, y)

        # Update nodes dict with new positions
        self.nodes = updated_nodes

        # Redraw all edges FIRST (so they appear behind nodes)
        for start_node, end_node in self.edges:
            if start_node in self.nodes and end_node in self.nodes:
                self.canvas.create_line(*self.nodes[start_node], *self.nodes[end_node], fill="blue")

        # Draw all nodes ON TOP (so they appear in front of edges)
        for name, (x, y) in self.nodes.items():
            # Draw node
            self.canvas.create_rectangle(x - self.canvas_rect_size, y - self.canvas_rect_size,
                                       x + self.canvas_rect_size, y + self.canvas_rect_size,
                                       fill='yellow', width=3, outline="blue")
            #print(f'font size:{self.canvas_font_size}')
            self.canvas.create_text(x, y, text=name, font=(self.pimgfont, self.canvas_font_size), fill="black") 
    
    def show_graph(self):
        print(nx.info(self.graph))

if __name__ == "__main__":
    
    config = OmegaConf.load('config.yaml')   
    root = tk.Tk()
    fine_root = tk.Tk()
    
    msg = f"""
-------------------CONTROLS--------------------------------------
    DOUBLE CLICK TO ADD A NODE (THEN SELECT THE NODE TYPE)
    CLICK AND DRAG BETWEEN 2 NODES TO ADD AN EDGE
    SCROLL WHEEL: Vertical panning
    ALT + SCROLL WHEEL: Horizontal panning
    +/- KEYS: Zoom in/out
    Alt+S: Save all images
---------ADD NODES FOR THE FOLLOWING REGIONS IN YOUR MAP---------""" +'\n'+ '\n'.join([f"{i+1}:{n}" for i,n in enumerate(config['node_types'])]) + '\n'+"""-- CONNECT THE NODES WITH EDGES OF THE FOLLOWING TYPES----------"""+ '\n'+'\n'.join([f"{i+1}:{n}" for i,n in enumerate(config['edge_types'])]) + '\n'+"""--------------------------------------------------------------------"""
    print(msg)
    app = SceneGraphBuilder(root, float(config['zoom_in']), float(config['zoom_out']),config['img'],config['scg'],config['out'],config['node_types'],config['edge_types'],
                            font_size=float(config['font_size']))  # Replace with your own image path
    root.mainloop()
