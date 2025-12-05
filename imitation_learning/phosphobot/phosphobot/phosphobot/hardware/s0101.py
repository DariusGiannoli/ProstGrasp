"""
Driver for S0101 4-motor robot with Feetech STS3215 servos
Motors: 2 (gripper2), 4 (wrist_pitch), 5 (wrist_roll), 6 (gripper1)
"""

from typing import Optional
import numpy as np
from loguru import logger

from phosphobot.hardware.base import BaseRobot

try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
    LEROBOT_AVAILABLE = True
except ImportError:
    logger.warning("lerobot not installed. S0101 hardware control will not work.")
    LEROBOT_AVAILABLE = False


class S0101Robot(BaseRobot):
    """
    S0101 robot avec 4 moteurs Feetech STS3215:
    - Motor 2: Gripper 2 (pince verte)
    - Motor 4: Wrist Pitch
    - Motor 5: Wrist Roll  
    - Motor 6: Gripper 1 (pince rouge)
    """
    
    name = "S0101"
    
    # Configuration des moteurs
    MOTOR_IDS = {
        "wrist_pitch": 4,
        "wrist_roll": 5,
        "gripper1": 6,
        "gripper2": 2,
    }
    
    # Noms des joints dans l'ordre pour phosphobot
    JOINT_NAMES = ["wrist_pitch", "wrist_roll", "gripper1", "gripper2"]
    
    @classmethod
    def from_port(cls, port):
        """
        Create S0101 from a serial port
        
        Args:
            port: ListPortInfo object from serial.tools.list_ports
            
        Returns:
            S0101Robot instance if port matches, None otherwise
        """
        if not LEROBOT_AVAILABLE:
            return None
            
        # Détecter les ports USB modem (caractéristique des Feetech)
        if "usbmodem" in port.device.lower():
            logger.info(f"Potential S0101 detected on {port.device}")
            return cls(port=port.device)
        
        return None
    
    def __init__(self, port: str = "/dev/tty.usbmodem58FD0170541", only_simulation: bool = False, **kwargs):
        """
        Initialize S0101 robot
        
        Args:
            port: Serial port (e.g., /dev/tty.usbmodem58FD0170541)
            only_simulation: If True, don't try to connect to hardware
        """
        self.port = port
        self.bus: Optional[FeetechMotorsBus] = None
        self._connected = False
        self._only_simulation = only_simulation
        
        if not LEROBOT_AVAILABLE and not only_simulation:
            raise ImportError("lerobot is required for S0101 hardware control. Install with: pip install lerobot")
        
        # Configuration des moteurs pour lerobot
        if LEROBOT_AVAILABLE:
            self.motors_config = {
                "wrist_pitch": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper1": Motor(6, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper2": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
            }
        
        # Appel au constructeur parent
        super().__init__(only_simulation=only_simulation, **kwargs)
        
    async def connect(self) -> bool:
        """Connect to the robot"""
        if self._only_simulation:
            self._connected = True
            logger.info("S0101 in simulation mode")
            return True
            
        try:
            logger.info(f"Connecting to S0101 on port {self.port}")
            
            # Créer le bus Feetech
            self.bus = FeetechMotorsBus(
                port=self.port,
                motors=self.motors_config,
            )
            
            self.bus.connect()
            self._connected = True
            
            # Activer les moteurs
            self.set_torque_enable(True)
            
            logger.success(f"Successfully connected to S0101 on {self.port}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to S0101: {e}")
            self._connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the robot"""
        if self.bus and self._connected:
            try:
                # Désactiver les moteurs avant de déconnecter
                self.set_torque_enable(False)
                self.bus.disconnect()
                logger.info("Disconnected from S0101")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
            finally:
                self._connected = False
    
    def is_connected(self) -> bool:
        """Check if robot is connected"""
        return self._connected
    
    def read_joint_positions(self) -> np.ndarray:
        """
        Read current joint positions
        
        Returns:
            Array of joint positions in radians [wrist_pitch, wrist_roll, gripper1, gripper2]
        """
        if self._only_simulation:
            return super().read_joint_positions()
            
        if not self._connected or not self.bus:
            logger.warning("Cannot read positions: not connected")
            return np.zeros(4)
        
        try:
            positions = []
            for name in self.JOINT_NAMES:
                # Lire la position en % (-100 à 100)
                pos_percent = self.bus.read("Present_Position", name)
                # Convertir en radians (approximatif: -100% = -π, 100% = π)
                pos_rad = (pos_percent / 100.0) * np.pi
                positions.append(pos_rad)
            
            return np.array(positions)
            
        except Exception as e:
            logger.error(f"Error reading joint positions: {e}")
            return np.zeros(4)
    
    def write_joint_positions(self, positions: np.ndarray, blocking: bool = False):
        """
        Write joint positions
        
        Args:
            positions: Array of joint positions in radians [wrist_pitch, wrist_roll, gripper1, gripper2]
            blocking: If True, wait for movement to complete
        """
        if self._only_simulation:
            return super().write_joint_positions(positions, blocking)
            
        if not self._connected or not self.bus:
            logger.warning("Cannot write positions: not connected")
            return
        
        try:
            for i, name in enumerate(self.JOINT_NAMES):
                # Convertir radians en pourcentage (-π à π → -100% à 100%)
                pos_rad = positions[i]
                pos_percent = (pos_rad / np.pi) * 100.0
                
                # Limiter à -100, 100
                pos_percent = np.clip(pos_percent, -100.0, 100.0)
                
                # Écrire la position
                self.bus.write("Goal_Position", name, pos_percent)
            
            if blocking:
                # Attendre que le mouvement soit terminé
                import time
                time.sleep(0.5)  # Ajuster selon la vitesse des moteurs
                
        except Exception as e:
            logger.error(f"Error writing joint positions: {e}")
    
    def get_joint_names(self) -> list:
        """Get list of joint names"""
        return self.JOINT_NAMES
    
    def get_joint_limits(self) -> tuple:
        """
        Get joint limits
        
        Returns:
            (lower_limits, upper_limits) in radians
        """
        # Limites approximatives basées sur le range -100 à 100%
        lower = np.array([-np.pi, -np.pi, -0.3, -0.3])  # gripper limits plus petits
        upper = np.array([np.pi, np.pi, 0.8, 0.8])
        return lower, upper
    
    def set_torque_enable(self, enable: bool):
        """Enable or disable motor torque"""
        if self._only_simulation:
            return
            
        if not self._connected or not self.bus:
            return
        
        try:
            for name in self.JOINT_NAMES:
                self.bus.write("Torque_Enable", name, 1 if enable else 0)
            logger.info(f"Torque {'enabled' if enable else 'disabled'}")
        except Exception as e:
            logger.error(f"Error setting torque: {e}")
    
    def __del__(self):
        """Cleanup on deletion"""
        self.disconnect()
