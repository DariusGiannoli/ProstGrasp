import { AlertCircle, Shield } from "lucide-react";

export function AIControlDisclaimer() {
  return (
    <div className="space-y-4">
      <div className="flex items-start gap-3 p-3 bg-green-50 dark:bg-green-950/30 border border-green-200 dark:border-green-800 rounded-md">
        <Shield className="size-5 text-green-600 mt-0.5 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-green-800 dark:text-green-300">
            Safety Precautions Required:
          </h4>
          <ul className="text-sm text-green-700 dark:text-green-400 mt-1 list-disc pl-5 space-y-1">
            <li>Clear the area of all obstacles, pets, and people</li>
            <li>Maintain direct visual contact with the robot at all times</li>
            <li>Keep the emergency stop button within immediate reach</li>
          </ul>
        </div>
      </div>

      <div className="flex items-start gap-3 p-3 bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-800 rounded-md">
        <AlertCircle className="size-5 text-gray-600 mt-0.5 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-gray-800 dark:text-gray-300">
            Liability Notice:
          </h4>
          <p className="text-sm text-gray-700 dark:text-gray-400 mt-1">
            By proceeding, you accept{" "}
            <span className="font-semibold">responsibility</span> for any damage
            to property or the robot itself.
          </p>
        </div>
      </div>
    </div>
  );
}
