import tkinter as tk
from tkinter import messagebox
import telnetlib
import time



tn = telnetlib.Telnet("192.168.43.145", 23)
custom_font=("Helvetica", 16)

class AlarmaApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Alarma App")
        self.root.configure(bg="#1E1E1E")  # Fondo oscuro
        width = 600
        height = 400
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - width) // 2
        y = (screen_height - height) // 2

        self.root.geometry(f"{width}x{height}+{x}+{y}")
        

        # Agrega un mensaje en la página principal
        title_label = tk.Label(root, text="¡ALARMA ALARMA!", font=("Helvetica", 22, "bold"), bg="#1E1E1E", fg="white")
        title_label.pack(pady=int(self.root.cget("height")) // 20)
        mensaje_label = tk.Label(root, text="Se ha detectado un intruso en su negocio", font=("Helvetica", 16), bg="#1E1E1E", fg="white")
        mensaje_label.pack(pady=int(self.root.cget("height")) // 30)
        self.activar_button = tk.Button(root, text="LLAMAR A LA TOMBA", command=self.hacer_cerrar, bg="#4CAF50", fg="white", font=custom_font)
        self.activar_button.pack(pady=20)

        self.desactivar_button = tk.Button(root, text="MUÑEQUE BURRO", command=self.desactivar_alarma, bg="#4CAF50", fg="white", font=custom_font)
        self.desactivar_button.pack(pady=10)
    def hacer_cerrar(self):
        tn.write(f"*".encode('utf-8'))
        self.root.destroy()
        recibir()

    def desactivar_alarma(self):
        response = messagebox.askquestion("Alarma", "todo bien mi fai?", icon='warning', parent=self.root)

        if response == 'yes':
            self.mostrar_teclado_numerico()
        elif response=='no':
            self.hacer_cerrar()

    def mostrar_teclado_numerico(self):
        num_pad_window = tk.Toplevel(self.root)
        num_pad_window.title("Teclado Numérico")
        num_pad_window.configure(bg="#1E1E1E")  # Fondo oscuro

        width = 400
        height = 400
        screen_width = num_pad_window.winfo_screenwidth()
        screen_height = num_pad_window.winfo_screenheight()
        x = (screen_width - width) // 2
        y = (screen_height - height) // 2

        num_pad_window.geometry(f"{width}x{height}+{x}+{y}")

        custom_font = ("Helvetica", 22, "bold")

        def boton_presionado(valor):
            tn.write(f"{valor}".encode('utf-8'))

        for i in range(1, 10):
            boton = tk.Button(num_pad_window, text=str(i), command=lambda i=i: boton_presionado(i), bg="#383838", fg="white", font=custom_font)
            boton.grid(row=(i - 1) // 3, column=(i - 1) % 3, padx=20, pady=10)

        boton_0 = tk.Button(num_pad_window, text="0", command=lambda: boton_presionado(0), bg="#383838", fg="white", font=custom_font)
        boton_0.grid(row=3, column=1, padx=20, pady=10)
        def cerrar():
            num_pad_window.destroy()
            tn.write(f"#".encode('utf-8'))
            self.root.destroy()
            recibir()

        boton_cerrar = tk.Button(num_pad_window, text="Cerrar", command=cerrar, bg="#FF4500", fg="white", font=custom_font)
        boton_cerrar.grid(row=4, column=1, pady=15)

def recibir():
    a=tn.read_until(b"Intruso")
    # Pausa el programa durante 2 segundos
    time.sleep(0.5)
    if a== b"Intruso":
         interfaz()

def interfaz():
    root = tk.Tk()
    root.configure(bg="#1E1E1E")  # Fondo oscuro
    app = AlarmaApp(root)
    root.mainloop()

recibir()



