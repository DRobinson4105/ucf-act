export type RideStatus =
  | "requested"
  | "assigned"
  | "arriving"
  | "in_progress"
  | "completed"
  | "cancelled";

export interface Vehicle {
  id: string;
  model: string;
  color: string;
  licensePlate?: string;
}

export interface Ride {
  id: string;
  convexId?: string;
  cartId?: string;
  pickupLocationId: string;
  dropoffLocationId: string;
  status: RideStatus;
  requestedAt: Date;
  pickupAt?: Date;
  completedAt?: Date;
  estimatedWaitTime?: number;
  vehiclePosition?: { latitude: number; longitude: number };
  vehicle: Vehicle;
  rating?: number;
  reviewComment?: string;
}

export interface Notification {
  id: string;
  title: string;
  message: string;
  timestamp: Date;
  read: boolean;
  type: "ride-update" | "system" | "promotion";
}
