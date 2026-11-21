#!/usr/bin/env python3
import pygame
import sys
import threading
import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

# --- CONFIGURATION ---
LIN_VEL_MAX = 2.0   
VERT_VEL_MAX = 1.2
YAW_RATE_MAX = 45.0
SMOOTHING = 0.15    # Facteur de lissage pour l'inertie

class DroneController:
    def __init__(self):
        # Vitesses cibles et réelles
        self.target_vx = self.target_vy = self.target_vz = self.target_yaw = 0.0
        self.vx = self.vy = self.vz = self.yaw = 0.0
        
        self.drone_state = "DECONNECTÉ"
        self.offboard_active = False
        self.is_running = True
        self.pending_action = None 

        # Lancement du thread drone
        self.drone_thread = threading.Thread(target=self.run_async_loop, daemon=True)
        self.drone_thread.start()

    def run_async_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.drone_main())

    async def drone_main(self):
        self.drone = System()
        await self.drone.connect(system_address="udp://:14540")
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.drone_state = "CONNECTÉ (SOL)"
                break
        
        while self.is_running:
            # ACTION : ARMER SEULEMENT (LES AILES TOURNENT)
            if self.pending_action == "ARM_ONLY":
                self.drone_state = "ARMEMENT..."
                await self.drone.action.arm()
                await asyncio.sleep(1)
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
                try:
                    await self.drone.offboard.start()
                    self.offboard_active = True
                    self.drone_state = "ARMÉ - PRÊT"
                except: pass
                self.pending_action = None

            # BOUCLE DE COMMANDE LISSÉE
            if self.offboard_active:
                self.vx += (self.target_vx - self.vx) * SMOOTHING
                self.vy += (self.target_vy - self.vy) * SMOOTHING
                self.vz += (self.target_vz - self.vz) * SMOOTHING
                self.yaw += (self.target_yaw - self.yaw) * SMOOTHING

                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(self.vx, self.vy, self.vz, self.yaw)
                )
                if abs(self.vz) > 0.1: self.drone_state = "EN VOL"

            # ACTION : ATTERRISSAGE
            if self.pending_action == "LAND":
                self.offboard_active = False
                try: await self.drone.offboard.stop()
                except: pass
                await self.drone.action.land()
                self.drone_state = "ATTERRISSAGE"
                self.pending_action = None

            await asyncio.sleep(0.05)

def main():
    pygame.init()
    screen = pygame.display.set_mode((700, 500))
    pygame.display.set_caption("PX4 ADVANCED DASHBOARD")
    
    # Couleurs et Polices
    CLR_BG = (20, 22, 30)
    CLR_PANEL = (35, 38, 50)
    CLR_ACCENT = (0, 200, 255)
    font_main = pygame.font.SysFont("Segoe UI", 24, bold=True)
    font_small = pygame.font.SysFont("Segoe UI", 18)
    
    ctrl = DroneController()
    clock = pygame.time.Clock()

    while ctrl.is_running:
        screen.fill(CLR_BG)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: ctrl.is_running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t: ctrl.pending_action = "ARM_ONLY"
                if event.key == pygame.K_l: ctrl.pending_action = "LAND"

        # Entrées Clavier
        keys = pygame.key.get_pressed()
        ctrl.target_vx = (keys[pygame.K_z] - keys[pygame.K_s]) * LIN_VEL_MAX
        ctrl.target_vy = (keys[pygame.K_d] - keys[pygame.K_q]) * LIN_VEL_MAX
        ctrl.target_vz = (keys[pygame.K_x] - keys[pygame.K_w]) * VERT_VEL_MAX
        ctrl.target_yaw = (keys[pygame.K_e] - keys[pygame.K_a]) * YAW_RATE_MAX

        # --- DESSIN DE L'INTERFACE ---
        
        # 1. Barre d'état supérieure
        pygame.draw.rect(screen, CLR_PANEL, (0, 0, 700, 60))
        status_color = (100, 255, 100) if "VOL" in ctrl.drone_state or "PRÊT" in ctrl.drone_state else (255, 100, 100)
        txt_state = font_main.render(f"DRONE STATUS: {ctrl.drone_state}", True, status_color)
        screen.blit(txt_state, (20, 15))

        # 2. Visualisation des vitesses (Dashboard style)
        def draw_v_bar(name, value, max_val, y_pos):
            txt = font_small.render(name, True, (200, 200, 200))
            screen.blit(txt, (40, y_pos))
            pygame.draw.rect(screen, (50, 55, 70), (150, y_pos+5, 200, 12))
            # Calcul de la largeur de la barre (centrée pour X/Y)
            width = (value / max_val) * 100
            pygame.draw.rect(screen, CLR_ACCENT, (250, y_pos+5, width, 12))
            pygame.draw.line(screen, (255,255,255), (250, y_pos), (250, y_pos+20), 2)

        draw_v_bar("PITCH (Vx)", ctrl.vx, LIN_VEL_MAX, 100)
        draw_v_bar("ROLL (Vy)", ctrl.vy, LIN_VEL_MAX, 140)
        draw_v_bar("THRUST (Vz)", -ctrl.vz, VERT_VEL_MAX, 180) # Inversé pour visuel "Up"
        draw_v_bar("YAW (R)", ctrl.yaw, YAW_RATE_MAX, 220)

        # 3. Panneau d'instructions
        pygame.draw.rect(screen, CLR_PANEL, (400, 100, 260, 350), border_radius=10)
        instr_title = font_main.render("COMMANDES", True, (255, 255, 255))
        screen.blit(instr_title, (420, 120))
        
        instructions = [
            ("[T] Démarrer moteurs", (0, 255, 150)),
            ("[W] Monter (Décoller)", (255, 255, 255)),
            ("[X] Descendre", (255, 255, 255)),
            ("[Z,S,Q,D] Translation", (255, 255, 255)),
            ("[A,E] Rotation", (255, 255, 255)),
            ("[L] Atterrir", (255, 100, 100)),
        ]
        
        for i, (text, color) in enumerate(instructions):
            img = font_small.render(text, True, color)
            screen.blit(img, (420, 170 + i*40))

        # 4. Indicateur de mode Offboard
        mode_txt = "OFFBOARD ACTIVE" if ctrl.offboard_active else "OFFBOARD STANDBY"
        mode_clr = (0, 255, 255) if ctrl.offboard_active else (100, 100, 100)
        screen.blit(font_small.render(mode_txt, True, mode_clr), (20, 460))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
