import { AppSidebar } from "@/components/common/app-sidebar";
import { SidebarProvider } from "@/components/ui/sidebar";

export function Sidebar() {
  return (
    <div className="hidden md:block w-[250px] border-r border-muted">
      <SidebarProvider defaultOpen={true}>
        <AppSidebar />
      </SidebarProvider>
    </div>
  );
}
